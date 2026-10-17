<?php
// 启用调试模式
error_reporting(E_ALL);
ini_set('display_errors', 1);

// 设置响应头
header('Content-Type: application/json');
header('Access-Control-Allow-Origin: *');
header('Access-Control-Allow-Methods: GET, POST, OPTIONS');
header('Access-Control-Allow-Headers: Content-Type');

// 处理预检请求
if ($_SERVER['REQUEST_METHOD'] === 'OPTIONS') {
    exit(0);
}

// 数据库配置
$servername = "localhost";
$username = "root01";
$password = "root01";
$dbname = "root01";

// 创建数据库连接
$conn = new mysqli($servername, $username, $password, $dbname);

if ($conn->connect_error) {
    die(json_encode([
        "status" => "error",
        "message" => "数据库连接失败: " . $conn->connect_error
    ]));
}

// 获取请求方法
$method = $_SERVER['REQUEST_METHOD'];

// 处理不同请求方法
try {
    switch ($method) {
        case 'GET':
            $response = handleGetRequest($conn);
            break;
            
        case 'POST':
            $response = handlePostRequest($conn);
            break;
            
        default:
            $response = [
                "status" => "error",
                "message" => "不支持的请求方法"
            ];
    }
    
    echo json_encode($response, JSON_PRETTY_PRINT);
    
} catch (Exception $e) {
    echo json_encode([
        "status" => "error",
        "message" => $e->getMessage(),
        "trace" => $e->getTraceAsString()
    ]);
} finally {
    $conn->close();
}

// 处理GET请求（完全保留原有数据结构和格式）
function handleGetRequest($conn) {
    // 检查是否是日志页面的特殊请求
    $isLogPage = isset($_GET['log_page']) && $_GET['log_page'] == '1';
    
    if ($isLogPage) {
        // 日志页面专用查询：只获取报警且未处理的记录
        $sql = "SELECT r.id, r.device_code, r.people_number, r.temp_people_num, r.Alarm, r.room_name, 
                    r.temperature_data, r.dl, r.created_at, r.is_processed,
                    r.processed_time, m.room_code as radar_id
                FROM radar_data r
                LEFT JOIN device_room_map m ON r.device_code = m.device_code
                WHERE r.Alarm = 0 AND r.is_processed = 0
                ORDER BY r.created_at DESC";
    } else {
        // 其他页面获取所有数据
        $sql = "SELECT r.id, r.device_code, r.people_number, r.temp_people_num, r.Alarm, r.room_name, 
                        r.temperature_data, r.dl, r.created_at, r.is_processed,
                        r.processed_time, m.room_code as radar_id
                FROM radar_data r
                LEFT JOIN device_room_map m ON r.device_code = m.device_code
                ORDER BY r.created_at DESC";
    }
    
    $result = $conn->query($sql);
    if ($result === false) {
        throw new Exception("查询radar_data失败: " . $conn->error);
    }

    $radarDataAll = $result->fetch_all(MYSQLI_ASSOC);
    $finalResponse = [];
    
    foreach ($radarDataAll as $radarData) {
        try {
            $radarId = $radarData['radar_id'] ?? $radarData['device_code'];
            
            $peopleLocationData = getPeopleLocationData($conn, $radarData['device_code']);
            
            // 完全保留原有数据处理逻辑
            
            
            // 完全保留原有数据结构，添加processed_time字段
            $finalResponse[] = [
                "radar_id" => $radarId,
                "device_code" => $radarData['device_code'],
                "room_name" => $radarData['room_name'],
                "people_number" => (int)$radarData['people_number'],
                "image_url" => "http://localhost/get_image.php?id=" . $radarData['id'],
                "x" => $peopleLocationData['x'],
                "y" => $peopleLocationData['y'],
                "z" => $peopleLocationData['z'],
                "Alarm" => (int)$radarData['Alarm'],
                "alarm_status" => ($radarData['Alarm'] == '0') ? '警报' : '正常',
                "deck" => substr($radarId, 0, 1),
                "dl" => isset($radarData['dl']) ? (float)$radarData['dl'] : 0.0,
                "time" => $radarData['created_at'],
                "is_processed" => (int)$radarData['is_processed'],
                "processed_time" => $radarData['processed_time'], // 新增字段
                "timestamp" => date('Y-m-d H:i:s'),
                "temp_people_num" => isset($radarData['temp_people_num']) ? (int)$radarData['temp_people_num'] : 0,
            ];
        } catch (Exception $e) {
            error_log("处理设备 {$radarData['device_code']} 时出错: " . $e->getMessage());
            continue;
        }
    }
    
    return ['data' => $finalResponse];
}

// 处理POST请求（修改为同时更新is_processed和processed_time字段）
function handlePostRequest($conn) {
    $json = file_get_contents('php://input');
    $data = json_decode($json, true);
    
    if (!isset($data['radar_id'])) {
        throw new Exception("缺少必要参数: radar_id");
    }

    // 通过radar_id获取device_code
    $device_code = getDeviceCode($conn, $data['radar_id']);
    if (!$device_code) {
        throw new Exception("找不到与舱号 {$data['radar_id']} 对应的设备");
    }
    
    // 获取处理时间，如果没有提供则使用当前时间
    $processed_time = isset($data['processed_time']) ? 
        $conn->real_escape_string($data['processed_time']) : 
        date('Y-m-d H:i:s');
    
    // 更新is_processed和processed_time字段
    $sql = "UPDATE radar_data SET is_processed = 1, processed_time = ?, Alarm = 1 
            WHERE device_code = ? AND Alarm = 0 AND is_processed = 0";
    
    $stmt = $conn->prepare($sql);
    if (!$stmt) {
        throw new Exception("预处理失败: " . $conn->error);
    }
    
    $stmt->bind_param("ss", $processed_time, $device_code);
    
    if (!$stmt->execute()) {
        throw new Exception("执行更新失败: " . $stmt->error);
    }
    
    if ($stmt->affected_rows === 0) {
        throw new Exception("没有记录被更新，可能记录已被处理或不存在");
    }
    
    return [
        "status" => "success",
        "message" => "报警已处理",
        "radar_id" => $data['radar_id'],
        "processed_time" => $processed_time,
        "affected_rows" => $stmt->affected_rows
    ];
}

// 完全保留原有辅助函数
function getDeviceCode($conn, $radar_id) {
    $sql = "SELECT device_code FROM device_room_map WHERE room_code = ? LIMIT 1";
    $stmt = $conn->prepare($sql);
    
    if (!$stmt) {
        throw new Exception("预处理失败: " . $conn->error);
    }
    
    $stmt->bind_param("s", $radar_id);
    
    if (!$stmt->execute()) {
        throw new Exception("执行查询失败: " . $stmt->error);
    }
    
    $result = $stmt->get_result();
    if ($row = $result->fetch_assoc()) {
        return $row['device_code'];
    }
    
    return null;
}

function getPeopleLocationData($conn, $device_code) {
    $sql = "SELECT x, y, z FROM people_location WHERE device_code = ?";
    $stmt = $conn->prepare($sql);
    
    if (!$stmt) {
        throw new Exception("预处理失败: " . $conn->error);
    }
    
    $stmt->bind_param("s", $device_code);
    
    if (!$stmt->execute()) {
        throw new Exception("执行查询失败: " . $stmt->error);
    }
    
    $result = $stmt->get_result();
    if ($row = $result->fetch_assoc()) {
        $x = !empty($row['x']) ? json_decode($row['x'], true) : [0];
        $y = !empty($row['y']) ? json_decode($row['y'], true) : [0];
        $z = !empty($row['z']) ? json_decode($row['z'], true) : [0];
        
        $x = is_array($x) ? $x : [$x];
        $y = is_array($y) ? $y : [$y];
        $z = is_array($z) ? $z : [$z];
        
        return [
            'x' => $x,
            'y' => $y,
            'z' => $z
        ];
    }
    
    return ['x' => [0], 'y' => [0], 'z' => [0]];
}
?>
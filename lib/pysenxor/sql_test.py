# coding=utf-8
import pymysql

dev_num_file_path = "dev_num.txt"
# cat /proc/cpuinfo | grep Serial
# RP-[项目代号]-[批次号]-[cpu序列号后8位]
dev_num_file = open(dev_num_file_path, 'r')
dev_num = dev_num_file.read()
# print(dev_num)

# 创建数据库连接
conn = pymysql.connect(
    host="192.168.0.200",
    port=3306,
    user="remote_user",
    password="remote_user01",
    database="firealert"
)

# 获取mysql服务信息（测试连接，会输出MySQL版本号）
print(conn.get_server_info())

# 创建游标对象，用于执行SQL查询
cursor = conn.cursor()

# 执行查询操作
cursor.execute('SELECT * FROM communication')

# 获取查询结果，返回元组
result = cursor.fetchall()
for res in result:
  if res[0] == dev_num:@
    print(res)
    if res[3] == 1:
      if res[1] == 1:
        # normal
        pass
      elif res[1] == 2:
        # danger
        pass
      elif res[1] == 3:
        # dangerstop
        pass
      sql = "UPDATE communication SET revise=%s WHERE device_code = %s"
      cursor.execute(sql, (0, dev_num))
      conn.commit()
      print(res)

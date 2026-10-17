def scale(x, srcRange, dstRange):
    return (x-srcRange[0])*(dstRange[1]-dstRange[0])/(srcRange[1]-srcRange[0])+dstRange[0]
    
print(int(scale(10, (-20, 20), (0, 40))))
print(int(scale(5, (0, 40), (0, 40))))
print(int(scale(1, (-10, 10), (0, 3))))

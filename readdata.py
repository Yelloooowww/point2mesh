import ctypes
import struct

lines = []
fp = open("formeshlab.txt", "w")
with open('catkin_ws/write_pcd_test.txt') as f:
    i = 0
    line = f.readline()
    while line:
        line = f.readline()
        x = line.split()
        if (i>10 and len(x)==4):
            test = int(x[3])
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)
            lines.append([x[0],x[1],x[2],r,g,b])
            wlines = [x[0],x[1],x[2],str(r/255.0),str(g/255.0),str(b/255.0)]
            fp.write(x[0]+" "+x[1]+" "+x[2]+" "+str(r)+" "+str(g)+" "+str(b)+"\n")
        i += 1


# print(lines)
fp.close()
f.close()

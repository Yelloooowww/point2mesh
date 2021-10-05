#!/usr/bin/env python3
import rospy
import numpy as np
import math
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import ctypes
import struct
import open3d as o3d
import time
from inputimeout import inputimeout, TimeoutOccurred
import pypcd

class save_pcd(object):
	def __init__(self):
		# ROS Subscriber
		self.sub_points = rospy.Subscriber("/rtabmap/cloud_map", PointCloud2, self.cb_points, queue_size=1)
		self.pc = o3d.geometry.PointCloud()
		self.msg = PointCloud2()
		print("save_pcd init done")



	def cb_points(self,msg):
		self.msg = msg
		# print("cb")


	def save_pcd_file(self):
		pc = pypcd.PointCloud.from_msg(self.msg)
		pc.save('foo.pcd', compression='binary_compressed')

		path = 'output.txt'

		msg = self.msg
		cloud_points = []
		cloud_colors = []
		gen = point_cloud2.read_points(msg, skip_nans=True)
		int_data = list(gen)
		with open(path, 'a') as f:
			for x in int_data:
				cloud_points.append([x[0],x[1],x[2]])
				test = x[3]
				# cast float32 to int so that bitwise operations are possible
				s = struct.pack('>f' ,test)
				i = struct.unpack('>l',s)[0]
				# you can get back the float value by the inverse operations
				pack = ctypes.c_uint32(i).value
				r = (pack & 0x00FF0000)>> 16
				g = (pack & 0x0000FF00)>> 8
				b = (pack & 0x000000FF)
				cloud_colors.append([r/255.0, g/255.0, b/255.0])
				# print r,g,b # prints r,g,b values in the 0-255 range
				#             # x,y,z can be retrieved from the x[0],x[1],x[2]

				lines = str(x[0])+" "+str(x[1])+" "+str(x[2])+" "+str(r/255.0)+" "+str(g/255.0)+" "+str(b/255.0)+"\n"
				f.write(lines)
		f.close()


		pcd = o3d.geometry.PointCloud()
		print(cloud_points,cloud_colors)
		pcd.points = o3d.utility.Vector3dVector(np.array(cloud_points))
		pcd.colors = o3d.utility.Vector3dVector(np.array(cloud_colors))
		self.pc = pcd
		# o3d.visualization.draw_geometries([self.pc])
		o3d.io.write_point_cloud(str(msg.header.stamp)+".pcd", pcd)
		print("save"+str(msg.header.stamp)+".pcd")


if __name__ == "__main__":
	rospy.init_node("kpfcnn")
	save_pcd_ = save_pcd()
	try:
		while True:
			try:
				k = inputimeout(prompt='>>', timeout=1)
				if k=="q":
					exit()
				if k=="s":
					save_pcd_.save_pcd_file()
			except TimeoutOccurred:
				k = "PASS"
				pass
	except KeyboardInterrupt:
		print('interrupted!')

	rospy.spin()

#!/usr/bin/env python
import rospy
import rosbag
import numpy 
import matplotlib.pyplot as plt
import rospkg
from geometry_msgs.msg import PoseStamped

class DataReducer():
	def __init__(self, n, file_name = "data/path_test.bag"):
		#read data points from rosbag and append them to numpy array		
		bag = rosbag.Bag(file_name)
		points = numpy.empty((0,7))
		
		for topic, msg, t in bag.read_messages(topics=['/vslam2d_pose']):
			points = numpy.append(points,[[msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]],axis = 0)
		
		bag.close()
		self.input = points
		print self.input[0,:]
		self.N = n
		self.output_raw = numpy.empty([0,2])
		self.output = numpy.empty([0,7])
	
	def fit(self):
		#using linear regression to fit the data
		#we switch the y and x axies to avoid multiple y values for the same x
		self.fit = numpy.polyfit(self.input[:,1],self.input[:,0],4)
		#get the polynomial function 
		poly = numpy.poly1d(self.fit)
		#getting N simples from the y axis and projecting them on the line to get the ouput points
		y_simple = numpy.linspace(numpy.amin(self.input[:,1]),numpy.amax(self.input[:,1]),self.N)
		self.output_raw = numpy.stack((poly(y_simple),y_simple), axis = -1)
		
	def display(self,output, with_input = True):
		if with_input:
			plt.plot(self.input[:,0],self.input[:,1],'.',output[:,0],output[:,1],'-')
		else:
			plt.plot(output[:,0],output[:,1],'.')
		plt.xlim(0,1)
		plt.ylim(numpy.amin(self.input[:,1]),numpy.amax(self.input[:,1]))
		plt.show()
	
	def point_matching(self):
		#we match the our original points to the fitted line ones
		#we calulate the distance between each line point and the original ones 
		#to find the closest original points presenting the same path
		self.output = numpy.empty([0,7])
		#loop through the n points from the fitted lines
		for point in self.output_raw:
			#calculate the distance between the point and the original input points
			distances = numpy.sqrt((self.input[:,1] - point[1])**2 + (self.input[:,0] - point[0])**2 )
			#add the point with the shortest distance to the path
			closest_point = self.input[numpy.argmin(distances),:]
			self.output= numpy.append(self.output,[(closest_point)], axis = 0)
	
	def save_results(self,file_name):
		#persisting the results in a bag
		bag = rosbag.Bag(file_name, 'w')
		try:
			for point in self.output:
				p = PoseStamped()
				p.pose.position.x = point[0]
				p.pose.position.y = point[1]
				p.pose.position.z = point[2]
				p.pose.orientation.x = point[3]
				p.pose.orientation.y = point[4]
				p.pose.orientation.z = point[5]
				p.pose.orientation.w = point[6]
				bag.write('/vslam2d_pose_reduced',p)
		finally:
			bag.close()
		pass

if __name__ == '__main__':
    try:
        #setup ros
	rospy.init_node('reducer_node',anonymous = True)
	rospack = rospkg.RosPack()
	current_path = rospack.get_path("line_finder")
	n = rospy.get_param('points_num')
	#we are fitting our data using a 4 deg polynominal 
	# having N equal to 5 or more we can get the polynominal path 
	#n = 15

	#set the input file and fit model
	data_reducer = DataReducer(n, file_name =  current_path+"/data/path_test.bag")
	data_reducer.fit()
	#we match the detected path points to the original one
	#in case we like to get back a path from the exact original points
	data_reducer.point_matching()
	data_reducer.display(data_reducer.output,with_input = False)
	#we save the results in results bag file
	data_reducer.save_results(file_name =  current_path+'/data/results.bag')
    except rospy.ROSInterruptException:
        pass

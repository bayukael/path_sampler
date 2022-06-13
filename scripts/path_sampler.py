#!/usr/bin/env python

import rospy
import rospkg
import rosbag
from geometry_msgs.msg import *
import numpy as np

if __name__ == '__main__':
	try:
		rospy.init_node('path_sampler')
		rospack = rospkg.RosPack()
		data_path = str(rospack.get_path('path_sampler')) + "/data/path_test.bag"
		sampled_output_path = str(rospack.get_path('path_sampler')) + "/data/sampled_output.bag"
		combined_output_path = str(rospack.get_path('path_sampler')) + "/data/combined_output.bag"
		
		sampled_topic_full_name = rospy.search_param('sampled_topic')
		data_path_full_name = rospy.search_param('data_path')
		num_of_samples_full_name = rospy.search_param('num_of_samples')
		
		sampled_topic = rospy.get_param(sampled_topic_full_name,'/vslam2d_pose')
		data_path_param = rospy.get_param(data_path_full_name,data_path)
		num_of_samples_param = rospy.get_param(num_of_samples_full_name,100)
		
		bag = rosbag.Bag(data_path_param)
		sampled_output_bag = rosbag.Bag(sampled_output_path, 'w')
		combined_output_bag = rosbag.Bag(combined_output_path, 'w')
		num_of_messages = bag.get_message_count('/vslam2d_pose')
		ctr = 0
		sample_idx = 0
		windowed_data = []
		for topic, msg, t in bag.read_messages(topics=[sampled_topic]):
			windowed_data.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
			if((ctr+1) == int(num_of_messages/num_of_samples_param*(sample_idx+1))):
				windowed_data_array = np.array(windowed_data)
				p = windowed_data_array[:,0:3]
				x_bar = p[:,0].mean()
				y_bar = p[:,1].mean()
				z_bar = p[:,2].mean()
				q = windowed_data_array[:,3:7] / windowed_data_array.shape[0]
				Q = np.zeros((4,4))
				np.matmul(q.transpose(),q,Q)
				w,v = np.linalg.eig(Q)
				w_largest = w[0]
				v_largest = v[:,0]
				for i in range(4):
					if w_largest < w[i]:
						w_largest = w[i]
						v_largest = v[:,i]
				pose_stamped = PoseStamped()
				pose_stamped.header = msg.header
				pose_stamped.header.seq = sample_idx
				pose_stamped.pose.position.x = x_bar
				pose_stamped.pose.position.y = y_bar
				pose_stamped.pose.position.z = z_bar
				pose_stamped.pose.orientation.w = v_largest[0]
				pose_stamped.pose.orientation.x = v_largest[1]
				pose_stamped.pose.orientation.y = v_largest[2]
				pose_stamped.pose.orientation.z = v_largest[3]
				sampled_output_bag.write(str(sampled_topic+"_sampled"),pose_stamped,pose_stamped.header.stamp)
				combined_output_bag.write(str(sampled_topic+"_sampled"),pose_stamped,pose_stamped.header.stamp)
				combined_output_bag.write(sampled_topic,msg,msg.header.stamp)
				windowed_data = []
				sample_idx += 1
			ctr += 1
		bag.close()
		sampled_output_bag.close()
		combined_output_bag.close()
		print("DONE!")
		print("number of messages processed: " + str(num_of_messages))
		print("number of samples generated: " + str(sample_idx))
		
	except rospy.ROSInterruptException:
		print ("Keyboard Interrupt")

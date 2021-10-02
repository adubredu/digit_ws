#/usr/bin/env python2
import rospy 
import tf 
import tf2_ros
import tf2_geometry_msgs
import time, sys
import numpy as np 
sys.path.append('../../digit_control/src')
from digit_control import Digit_Control
from digit_control import BasicClient
from apriltag_ros.msg import AprilTagDetectionArray
from digit_manipulation import Digit_Manipulation

class Grasp_Fiducial:
	def __init__(self,dm):
		# rospy.init_node("grasp_fiducial")
		self.dm = dm
		self.tag_poses={} 
		self.bottle_grasp_pose = [0.37,-0.095, -0.01]
		self.glass_grasp_pose = [0.37, 0.128, -0.02]
		self.get_bottle_offsets()
		self.get_glass_offsets()
		self.obj_offsets= {'bottle':self.bot_offsets, 
						   'glass':self.glass_offsets}
		rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_callback)

	def tag_callback(self, data):
		if len(data.detections) > 0:
			for tag in data.detections:
				self.tag_poses[tag.id[0]] = tag.pose.pose

	def get_bottle_offsets(self):
		bpg = self.bottle_grasp_pose
		self.bot_offsets = {1: [bpg[0]-0.954, bpg[1]-0.332, bpg[2]--0.339],
							2: [bpg[0]-0.947, bpg[1]--0.312, bpg[2]--0.341],
							3: [bpg[0]-0.5, bpg[1]-0.333, bpg[2]--0.343],
							4: [bpg[0]-0.499, bpg[1]--0.332, bpg[2]--0.373]}

	def get_glass_offsets(self):
		gpg = self.glass_grasp_pose
		self.glass_offsets = {1: [gpg[0]-0.9643, gpg[1]-0.261, gpg[2]--0.3486],
							  2: [gpg[0]-0.9725, gpg[1]--0.3836, gpg[2]--0.3399],
							  3: [gpg[0]-0.5, gpg[1]-0.2422, gpg[2]--0.3329],
							  4: [gpg[0]-0.5285, gpg[1]--0.4226, gpg[2]--0.3725] }

	def print_fids(self):
		print(self.tag_poses[1])

	def get_marker_pos_in_base_frame(self, marker_pose):
		tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
		tf_listener = tf2_ros.TransformListener(tf_buffer)
		transform = tf_buffer.lookup_transform('base_link', 
			"forward_chest_realsense_d435_color_optical_frame",rospy.Time(0), rospy.Duration(1.0))
		transed_pose = tf2_geometry_msgs.do_transform_pose(marker_pose, transform)

		return transed_pose

	def print_transformed_fids(self):
		for idx in self.tag_poses:
			p = self.tag_poses[idx]
			pose = self.get_marker_pos_in_base_frame(p)
			print(idx," : ")
			print(pose)
			print("*"*20)

	def get_grasp_pose(self, objectname):
		obj_offset = self.obj_offsets[objectname]
		if len(self.tag_poses) > 0:
			for tagid in self.tag_poses:
				p = self.tag_poses[tagid]
				mpt = self.get_marker_pos_in_base_frame(p)
				bot_pose = [mpt.pose.position.x + obj_offset[tagid][0],
							mpt.pose.position.y + obj_offset[tagid][1],
							mpt.pose.position.z + obj_offset[tagid][2]]
				print(bot_pose)
				return bot_pose
		else:
			print("NO FIDUCIAL DETECTED!")
			return None

	def drink_pouring_demo(self):
		print('picking bottle')
		bottle_grasp_pose = self.get_grasp_pose('bottle')
		self.dm.side_pick_right(bottle_grasp_pose,'right')
		prev_pose = self.dm.raise_up(bottle_grasp_pose,'right')
		time.sleep(5)

		print('picking glass')
		glass_grasp_pose = self.get_grasp_pose('glass')
		self.dm.side_pick_left(glass_grasp_pose, 'left', bimanual=True, prevarmname='right', prevarmpose=prev_pose)
		prev_pose = self.dm.raise_up(glass_grasp_pose, 'left', bimanual=True, prevarmname='right', prevarmpose=prev_pose)
		time.sleep(5)
		
		self.dm.dc.move_gripper_to_conf([60, self.dm.none], 'right')
		time.sleep(10)
		self.dm.dc.move_gripper_to_conf([130, self.dm.none], 'right')
		time.sleep(5)

		print('putting obs down')
		prev_pose = self.dm.put_down(bottle_grasp_pose, 'right', bimanual=True, prevarmname='left', prevarmpose=prev_pose )
		time.sleep(5)
		prev_pose = self.dm.put_down(glass_grasp_pose, 'left', bimanual=True, prevarmname='right', prevarmpose=prev_pose )
		time.sleep(5)


		print('Going home...')
		self.dm.move_to_init('right', bimanual=True, prevarmname='left', prevarmpose=prev_pose)
		time.sleep(5)
		self.dm.move_to_init('left')
		








if __name__ == "__main__":   

	ws = BasicClient('ws://10.10.2.1:8080', protocols=['json-v1-agility'])
	# ws = BasicClient('ws://127.0.0.1:8080', protocols=['json-v1-agility'])
	ws.daemon = False

	while True:
		try:
			ws.connect()
			print("WS connection established")
			time.sleep(1)
			break
		except:
			print('WS connection NOT established')
			time.sleep(1) 

	dc = Digit_Control(ws)
	dm = Digit_Manipulation(dc)
	gf = Grasp_Fiducial(dm)
	time.sleep(10) #to allow for fiducial detection
	# gf.print_transformed_fids() 
	gf.drink_pouring_demo()
	rospy.spin()
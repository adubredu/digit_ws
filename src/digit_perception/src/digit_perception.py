import rospy, os, sys, time
from sensor_msgs.msg import Image
import cv2, copy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from detecto import core, utils, visualize
from PIL import Image as Img
import matplotlib.pyplot as plt

class Digit_Perception:
	def __init__(self):
		rospy.Subscriber('/forward_chest_realsense_d435/color/image_raw', Image, self.camera_callback)
		self.camera_feed = None
		self.obs = None
		self.obj_list = ['banana', 'cube', 'sponge','lion', 'chlorox','tea']
		self.model = core.Model.load('/home/alphonsus/research/digit/digit_ws/src/digit_perception/perception_models/detector_v3.pth', self.obj_list)
		self.bridge = CvBridge()
		self.data_ind = 1

	def camera_callback(self, image_data): 
		img = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width, -1)
		imge = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		self.obs = Img.fromarray(img)  #for predictions
		self.camera_feed = imge #for display
		self.disp = copy.deepcopy(imge)

	def detect_objects(self):
		if self.obs is not None:
			predictions = self.model.predict(self.obs)
			labels, boxes, scores = predictions
			boxes = boxes.numpy()
			scores = scores.numpy()
			num_observed = len(labels)
			observed = {}
			preds = []
			idd = 0
			for i in range(num_observed):
				dicts = {'name':labels[i],
						 'id':idd,
						'coordinates': boxes[i],
						 'confidence':scores[i],
						 'color': (np.random.randint(255),\
								np.random.randint(255),\
								np.random.randint(255))
						 }
				preds.append(dicts)
				observed[labels[i]] = dicts
				idd+=1

			clusters = {}
			for box in preds:
				fit = False
				mid = ((box['coordinates'][0] + box['coordinates'][2])/2, \
					(box['coordinates'][1] + box['coordinates'][3])/2)

				for key in clusters:
					kcd = key.split('_')
					ikcd = [float(i) for i in kcd]
					dist = np.sqrt((ikcd[0] - mid[0])**2 + (ikcd[1]-mid[1])**2)
					if dist < 15:
						clusters[key].append(box)
						fit = True
						break
				if not fit:
					clusters[str(mid[0])+'_'+str(mid[1])] = [box] 
			scene = {} 
			for key in clusters:
				weights = [b['confidence'] for b in clusters[key]]
				maxind = np.argmax(weights) 
				maxbox = clusters[key][maxind]  

				if maxbox['confidence']>0.1 and maxbox['coordinates'][1]>70:
					name = maxbox['name']
					if name not in scene: 
						scene[name] = [(box['name'], box['confidence'], \
							[int(b) for b in box['coordinates']]) for box in clusters[key]]
					else:
						scene[name].append((name, maxbox['confidence'], [int(b) for b in maxbox['coordinates']]))


			norm_scene = self.normalize_weights(scene)
			self.scene_belief = self.compute_scene_belief(norm_scene) 
			return norm_scene 


	def compute_scene_belief(self, scene):
		num_objects = len(self.obj_list) 
		beliefs = []
		for item in scene:
			hypotheses = []; iih=[]; wih=[]
			cd = scene[item][0][2]
			gx = int((cd[0]+cd[2])/2); gy = int((cd[1]+cd[3])/2) 
			for hypothesis in scene[item]: 
				s = [hypothesis[0], hypothesis[1]]
				hypotheses.append(s)
			
			beliefs.append(hypotheses)

		return beliefs


	def normalize_weights(self, scene):
			norm_scene = {} 
			for item in scene:
				names=[]; weights=[];coord=[]; ids=[]
				norm_scene[item]=[]
				for name,wt,cd in scene[item]: 
					if name not in names:
						names.append(name)
						weights.append(wt)
						coord.append([int(c) for c in cd]) 
				
				rest = len(self.obj_list)-len(weights)
				if rest != 0:
					p = np.abs(1-np.sum(weights))/(len(self.obj_list)-len(weights))
					for i in range(rest):
						weights.append(p)
				summ = np.sum(weights)
				norm_wt = weights/summ
				for n in self.obj_list:
					if n not in names:
						names.append(n)
						coord.append(coord[-1]) 
				if len(names) > 0:
					for name, wt, cd in zip(names, norm_wt, coord):
						norm_scene[item].append((name,wt,cd))
				else:
					norm_scene.pop(item, None) 
			return norm_scene


	def annotate_object_beliefs(self, scene): 
		camera_view = self.camera_feed
		for item in scene:
			nm, cf, cd = scene[item][0]
			color = (np.random.randint(255),\
					np.random.randint(125),\
					np.random.randint(100))
			camera_view = cv2.rectangle(camera_view, (int(cd[0]),
			 int(cd[1])), (int(cd[2]), int(cd[3])),color , 1)
			cv2.putText(camera_view, nm+':'+str(round(cf,2)), (int(cd[0]),int(cd[1])-10),\
				cv2.FONT_HERSHEY_SIMPLEX, 0.5, color,2)
		return camera_view


	def run_detection(self):
		while not rospy.is_shutdown():
			if self.camera_feed is not None:  
				belief = self.detect_objects()
				camera_view = self.annotate_object_beliefs(belief)
				# cv2.imshow('Test',self.disp)
				# if cv2.waitKey(1) & 0xFF == ord('q'):
				#     break 
				yield belief

if __name__ == "__main__":
	rospy.init_node('digit_perception')
	try:
		dp = Digit_Perception()
		while not rospy.is_shutdown():
			bel = next(dp.run_detection())
			print(bel)
			print('')
	except KeyboardInterrupt:
		rospy.loginfo('Shutting down')
	rospy.spin()

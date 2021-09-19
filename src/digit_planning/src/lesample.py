import os, sys, time  
import numpy as np   
from planning_utils import Planning_Utils  

class Lesample:
	def __init__(self, dm, dp, domain_path):
		self.dm = dm
		self.dp = dp
		self.pu = Planning_Utils(dm,dp)
		self.domain_path = domain_path 
		self.terminate = False 
		self.packed = []
		self.num_items = 5



	def perform_lesample(self): 
		while not (len(self.packed)==self.num_items) or not self.terminate: 
			inboxlist, pilelist, lightlist, heavylist =  \
				self.sample_belief_space() 
			problem_path, alias = self.pu.create_pddl_problem(inboxlist, pilelist, lightlist, heavylist)  
			self.run_planner(problem_path, alias) 
		print('FIN')


	def run_planner(self, problem_path, alias): 
		plan = self.pu.symbolic_planner_plan()  
		
		if plan is None or len(plan) < 1:
			print('No valid plan found')
			return

		for action in plan: 
			result = self.pu.execute_action(action, alias)  
			if not result:
				print('REPLANNING') 
				inboxlist, pilelist, lightlist, heavylist =  \
				self.sample_belief_space() 
				new_problem_path, newalias = self.pu.create_pddl_problem(inboxlist, pilelist, lightlist, heavylist)  
				self.run_planner(new_problem_path, newalias)
				break
			else:
				if action[0] == 'put-in-box':
					self.packed.append(alias[action[1]])
			
		return 


	def sample_belief_space(self):
		lightlist, heavylist = [],[]   
		pilelist, _ = self.sample(); print('sees: ',pilelist)
		if len(pilelist)==0: self.terminate = True
		np.random.shuffle(pilelist)
		inboxlist = self.packed
		for b in inboxlist:
			if b in pilelist:
				pilelist.remove(b)
		for itemname in inboxlist+pilelist:
			mass = self.pu.get_item_mass(itemname)
			if mass == 'heavy':
				heavylist.append(itemname)
			else:
				lightlist.append(itemname)
		return inboxlist, pilelist, lightlist, heavylist


	def sample(self):
		belief = next(self.dp.run_detection()) 
		sampled_items=[]
		item_weights = [] 
		print(belief)
		for bu in belief: 
			bunch = belief[bu]
			items = [b[0] for b in bunch]
			norm_weights = [b[1] for b in bunch]
			norm_weights = np.array(norm_weights)/np.sum(norm_weights) 
			sample = np.random.choice(items, size=1, p=norm_weights) 
			ind = items.index(sample[0]) 
			item_weights.append(norm_weights[ind])
			sampled_items.append(sample[0]) 
		return sampled_items, item_weights

import numpy as np
from fd import Fast_Downward
import os, time

class Planning_Utils:
    def __init__(self, dm, dp):
        self.dm = dm 
        self.dp = dp 
        self.holding = None
        self.full_cpty = 6
        self.masses = {"banana":"heavy", 
                       "lion":"light",
                       "sponge":"light",
                       "cube":"light",
                       "chlorox":"heavy",
                       "tea":"heavy"}
        # sponge =  [0.5,-0.23,0.025, 150] 
        # chlorox =  [0.45,-0.3,0.025, 135]
        # tea =  [0.4,-0.19,0.022, 135] 
        # monkey =  [0.31,-0.26,0.015, 170]
        # banana =  [0.31,-0.22,0.014, 150] 
        # lion =   [0.42,-0.26,0.022, 150]

        # placepose1 = [0.45,0.13,0.05, 260]   
        # placepose2 = [0.45,0.05,0.05,270]  
        # placepose3 = [0.45,0.0,0.05,270] 
        # placepose4 = [0.45,0.13,0.08, 260]   
        # placepose5 = [0.45,0.05,0.08,270]  
        # placepose6 = [0.45,0.0,0.08,270] 

        sponge =  [0.5,-0.23,0.025, 150] 
        chlorox =  [0.45,-0.3,0.025, 135]
        tea =  [0.4,-0.16,0.022, 135] 
        monkey =  [0.29,-0.26,0.015, 170]
        banana =  [0.31,-0.21,0.014, 150] 
        lion =   [0.42,-0.26,0.022, 150]

        placepose1 = [0.4,0.13,0.05, 260]   
        placepose2 = [0.4,0.05,0.05,270]  
        placepose3 = [0.4,0.0,0.05,270] 
        placepose4 = [0.4,0.13,0.08, 260]   
        placepose5 = [0.4,0.05,0.08,270]  
        placepose6 = [0.4,0.0,0.08,270] 


        self.obs = {"sponge":sponge, "lion":lion, "banana":banana, 
                     "cube":monkey, "chlorox":chlorox, "tea":tea}
        self.items = ["banana", "sponge", "cube", "lion","banana","chlorox", "tea"]


        self.places = [placepose1, placepose2, placepose3, placepose4, placepose5, placepose6]

    def create_pddl_problem(self,inbox, topfree, mediumlist, heavylist):
        topfree = list(set(topfree)) 
        heavylist = list(set(heavylist))
        mediumlist = list(set(mediumlist))
        holding = self.holding
        itlist = heavylist+mediumlist
        alias = {}
        hc = 0
        for item in heavylist:
            alias[item] = 'h'+str(hc)
            hc+=1

        mc = 0
        for item in mediumlist:
            alias[item] = 'm'+str(mc)
            mc+=1

        if holding is not None:
            name = holding
            if self.masses[name] == 'heavy':
                alias[name] = 'h'+str(hc)
                hc+=1
            else:
                alias[name] = 'm'+str(mc)
                mc+=1

        init = "(:init (handempty) "
        if holding is not None:
            init = "(:init (holding "+alias[holding]+") "

        for item in inbox:
            init += "(inbox "+alias[item]+") "  
            init += "(topfree "+alias[item]+") "


        for item in topfree: 
            init += "(topfree "+alias[item]+") "
            init += "(inclutter "+alias[item]+") "


        init +=  ")\n"    

        goal = "(:goal (and "
        for h in heavylist[:self.full_cpty]:
            goal += "(inbox "+alias[h]+") "

        hleft = heavylist[self.full_cpty:]
        hin = heavylist[:self.full_cpty]
        hputon = hin[:len(hleft)]

        for l, i in zip(hleft, hputon):
            goal += "(on "+alias[l]+" "+alias[i]+") "


        hvlist_free = hleft + hin[len(hleft):]
            
        mlen=len(mediumlist)
        hlen=len(hvlist_free)
        stop = self.full_cpty - hlen

        if hlen >= self.full_cpty and mlen > hlen:
            for m,h in zip(mediumlist[:hlen],hvlist_free):
                goal += "(on "+alias[m]+" "+alias[h]+") "

            lenontop = len(mediumlist[:hlen])
            for m, mm in zip(mediumlist[hlen:][:lenontop], mediumlist[:hlen]):
                goal += "(on "+alias[m]+" "+alias[mm]+") "

            # lenleft
        else:
            for m in mediumlist[:stop]:
                goal += "(inbox "+alias[m]+") "

            currfreeinbox = hvlist_free+mediumlist[:stop]
            for m, mh in zip(mediumlist[stop:(stop+self.full_cpty)], currfreeinbox):
                goal +="(on "+alias[m]+" "+alias[mh]+") "

            left = mediumlist[(stop+self.full_cpty):]
            newcurron = mediumlist[stop:(stop+self.full_cpty)]
            for m, mm in zip(left, newcurron[:len(left)]):
                goal +="(on "+alias[m]+" "+alias[mm]+") "

        goal+=")))\n"

        definition = "(define (problem PACKED-GROCERY) \n(:domain GROCERY) \n (:objects "
        for al in alias.values():
            definition += al+" "
        definition += "- item)\n"

        problem = definition + init + goal
        prob_path =  "/home/alphonsus/research/digit/digit_ws/src/digit_planning/pddl/newprob.pddl"
        f = open(prob_path,"w")
        f.write(problem)
        f.close()
        dir_path = os.path.dirname(os.path.realpath(__file__))  
        swapped_alias  = dict([(value, key) for key, value in alias.items()]) 
        return prob_path, swapped_alias


    def symbolic_planner_plan(self):
        f = Fast_Downward()
        plan = f.plan('/home/alphonsus/research/digit/digit_ws/src/digit_planning/pddl/domain.pddl', '/home/alphonsus/research/digit/digit_ws/src/digit_planning/pddl/newprob.pddl')
        if plan is not None and len(plan) != 0: 
            return plan 
        return None

    def execute_action(self, action, alias):
        try:
            print(action[0], alias[action[1]])
            success = True
            if action[0] == 'pick-from-clutter':
                proposed_name = alias[action[1]]
                success = self.pick_up(proposed_name)
                success = success and self.validate(proposed_name) 

            elif action[0] == 'pick-from-box':
                proposed_name = alias[action[1]]
                success = self.pick_from_bag(proposed_name) 

            elif action[0] == 'pick-from':
                proposed_name = alias[action[1]]
                success = self.pick_up(proposed_name)
                success = success and self.validate(proposed_name) 


            elif action[0] == 'put-in-box':
                proposed_name = alias[action[1]]
                success = self.put_in_box(proposed_name)
                success = success and self.validate(proposed_name) 


            elif action[0] == 'put-in-clutter':
                proposed_name = alias[action[1]]
                success = self.put_in_clutter(proposed_name)
                success = success and self.validate(proposed_name) 


            elif action[0] == 'put-on':
                proposed_name = alias[action[1]]
                success = self.put_in_box(proposed_name)
                success = success and self.validate(proposed_name) 

        except Exception as e: 
            print('execute exception:',e)
            return True 
        
        return success


    def get_item_mass(self, itemname):
        return self.masses[itemname]

    def validate(self, proposed_name):
        return True

    def pick_up(self, name):
        self.holding = name
        # print("Picking ",name)
        self.dm.pick_object(self.obs[name], "right")
        time.sleep(5)

    def pick_from_bag(self, name):
        self.holding = name
        # print("Picking ",name)
        self.dm.pick_object(self.places[2], "right")
        time.sleep(5)


    def put_in_box(self, name):
        self.holding = None
        # print("Putting ",name)
        pos = self.places.pop()
        self.dm.place_object(pos, "right")
        self.items.remove(a)
        time.sleep(5)


    def put_in_clutter(self, name):
        self.holding = None
        # print("Putting ",name)
        self.dm.place_object(self.obs["sponge"], "right")
        time.sleep(5)



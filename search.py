import re
import heapq 
import copy
import sys
import math
from queue import *

sys.setrecursionlimit(100000)

#GlobeState objects are used to traverse the state graph
class GlobeState:
    
    def __init__(self, axis0_180, axis90_270, equator, g, path):
        self.axis0_180 = axis0_180
        self.axis90_270 = axis90_270
        self.equator = equator
        self.g = g
		self.path = path
        
    #rotate clockwise axis 0-180
    def RC_axis0_180(self):
        newD = {}
        
        newD[(30,180)] = self.axis0_180[(0,0)]
        
        for i in [60, 90, 120, 150, 180]:
            newD[(i, 180)] = self.axis0_180[(i-30,180)]
        
        newD[(150,0)] = self.axis0_180[(180,180)]
        
        for i in [120, 90, 60, 30, 0]:
            newD[(i, 0)] = self.axis0_180[(i+30,0)]
        
        #swaps in axis90_270    
        self.axis90_270[(0,0)] = self.axis0_180[(30,0)]
        self.axis90_270[(180,180)] = self.axis0_180[150,180]
        
        #swaps in equator
        self.equator[(90,0)] = self.axis0_180[120,0]
        self.equator[(90,180)] = self.axis0_180[60,180]
        
        #new axis0_180
        self.axis0_180 = newD
    
    def RCC_axis0_180(self):
        newD = {}
        
        newD[(0,0)] = self.axis0_180[(30,180)]
        
        for i in [30, 60, 90, 120, 150]:
            newD[(i,0)] = self.axis0_180[(i-30,0)]
        
        newD[(180, 180)] = self.axis0_180[(150,0)]
        
        for i in [150, 120, 90, 60, 30]:
            newD[(i,180)] = self.axis0_180[(i+30, 180)]
            
        #swaps in axis90_270
        self.axis90_270[(0,0)] = self.axis0_180[(30,180)]
        self.axis90_270[(180, 180)] = self.axis0_180[(150, 0)]
        
        #swaps in equator
        self.equator[(90,0)] = self.axis0_180[(60,0)]
        self.equator[(90,180)] = self.axis0_180[(120, 180)]
        
        #new axis0_180
        self.axis0_180 = newD
    
    def RC_axis90_270(self):
        newD = {}
        
        newD[(30,270)] = self.axis90_270[(0,0)]
        
        for i in [60, 90, 120, 150]:
            newD[(i,270)] = self.axis90_270[(i-30, 270)]
        
        newD[(180,180)] = self.axis90_270[(150,270)]
        newD[(150, 90)] = self.axis90_270[(180,180)]
        
        for i in [120, 90, 60, 30]:
            newD[(i, 90)] = self.axis90_270[(i+30,90)]
        
        newD[(0,0)] = self.axis90_270[(30,90)]
        
        #swaps in axis0_180
        self.axis0_180[(0,0)] = self.axis90_270[(30,90)]
        self.axis0_180[(180,180)] = self.axis90_270[(150,270)]
        
        #swaps in equator
        self.equator[(90,90)] = self.axis90_270[(120,90)]
        self.equator[(90,270)] = self.axis90_270[(60,270)]
        
        #new axis90_270
        self.axis90_270 = newD
    
    def RCC_axis90_270(self):
        newD = {}
        
        newD[(0,0)] = self.axis90_270[(30, 270)]
        newD[(30,90)] = self.axis90_270[(0,0)]
        
        for i in [60, 90, 120, 150]:
            newD[(i,90)] = self.axis90_270[(i-30,90)]
            
        newD[(180,180)] = self.axis90_270[(150,90)]
        newD[(150, 270)] = self.axis90_270[(180,180)]
        
        for i in [120, 90, 60, 30]:
            newD[(i,270)] = self.axis90_270[(i+30, 270)]
        
        #sawps in axis0_180
        self.axis0_180[(0,0)] = self.axis90_270[(30, 270)]
        self.axis0_180[(180,180)] = self.axis90_270[(150, 90)]
        
        #swaps in equator
        self.equator[(90,90)] = self.axis90_270[(60,90)]
        self.equator[(90,270)] = self.axis90_270[(120,270)]
        
        #new axis90_270
        self.axis90_270 = newD
    
    def RC_equator(self):
        newD = {}
        
        newD[(90,330)] = self.equator[(90,0)]
        
        for i in [300, 270, 240, 210, 180, 150, 120, 90, 60, 30, 0]:
            newD[(90, i)] = self.equator[(90, i+30)]
        
        #swaps in axis0_180
        self.axis0_180[(90,0)] = self.equator[(90, 30)]
        self.axis0_180[(90,180)] = self.equator[(90, 210)]
        
        #swaps in axis90_270
        self.axis90_270[(90, 90)] = self.equator[(90, 120)]
        self.axis90_270[(90, 270)] = self.equator[(90, 300)]
        
        #new equator
        self.equator = newD
        
    def RCC_equator(self):
        newD = {}
        
        newD[(90,0)] = self.equator[(90,330)]
        
        for i in [30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330]:
            newD[(90, i)] = self.equator[(90, i-30)]
        
        #swaps in axis0_180
        self.axis0_180[(90,0)] = self.equator[(90, 330)]
        self.axis0_180[(90,180)] = self.equator[(90, 150)]
        
        #swaps in axis90_270
        self.axis90_270[(90, 90)] = self.equator[(90, 60)]
        self.axis90_270[(90, 270)] = self.equator[(90, 240)]
        
        #new equator
        self.equator = newD
        
    def getCurrentHeuristic(self):
        H = 0
        
        for key in self.axis0_180.keys():
            H = H + heuristic(key, self.axis0_180[key])
        
        for key in self.axis90_270.keys() - self.axis0_180.keys():
            H = H + heuristic(key, self.axis90_270[key])
        
        for key in self.equator.keys() - self.axis90_270.keys() - self.axis0_180.keys():
            H = H + heuristic(key, self.equator[key])
        
        return H/12
    
    
    def isSolved(self):
        flag = True
        for key in self.axis0_180.keys():
            if (key != self.axis0_180[key]):
                flag = False
        for key in self.axis90_270.keys():
            if (key != self.axis90_270[key]):
                flag = False
        for key in self.equator.keys():
            if (key != self.equator[key]):
                flag = False
        
        return flag
    
    

threeDimMap = {
    (0,0) : (1,1,2),
    (30,0) : (1, 1.5, 1.866),
    (60,0) : (1, 1.866, 1.5),
    (90,0) : (1, 2, 1),
    (120,0) : (1, 1.866, 0.5),
    (150,0) : (1, 1.5, 0.144),
    (180,180) : (1, 1, 0),
    (150,180) : (1, 0.5, 0.144),
    (120,180) : (1, 0.144, 0.5),
    (90,180) : (1, 0, 1),
    (60,180) : (1, 0.144, 1.5),
    (30,180) : (1, 0.5, 1.866),
    
    #(0,0) : (1,1,2)
    (30,270) : (1.5, 1, 1.866),
    (60,270) : (1.866, 1, 1.5),
    (90,270) : (2, 1, 1),
    (120,270) : (1.866, 1, 0.5),
    (150,270) : (1.5, 1, 0.144),
    #(180,180) : (1,1,0)
    (150,90) : (0.5, 1, 0.144),
    (120,90) : (0.144, 1, 0.866),
    (90,90) : (0, 1, 1),
    (60,90) : (0.144, 1, 1.5),
    (30,90) : (0.5, 1, 1.866),
    
    #(90,0) : (1,2,1)
    (90,330) : (1.5, 1.866, 1),
    (90,300) : (1.866, 1.5, 1),
    #(90,270) : (2,1,1)
    (90,240) : (1.866, 0.866, 1),
    (90,210) : (1.5, 0.5, 1),
    #(90,180) : (1,0,1)
    (90,150) : (0.5, 0.5, 1),
    (90,120) : (0.144, 0.866, 1),
    #(90,90) : (0,1,1)
    (90,60) : (0.144, 1.5, 1),
    (90,30) : (0.5, 1.866, 1)
    
}

axis0_180Base=[(0, 0),(30, 0),(60, 0),(90, 0),(120, 0),(150, 0),(180, 180),(150, 180),(120, 180),(90, 180),(60, 180),(30, 180)]

axis90_270Base=[(0, 0),(30, 90),(60, 90),(90, 90),(120, 90),(150, 90),(180, 180),(150, 270),(120, 270),(90, 270),(60, 270),(30, 270)]

equatorBase = [(90,0),(90,30),(90,60),(90,90),(90,120),(90,150),(90,180),(90,210),(90,240),(90,270),(90,300),(90,330)]

#define heuristic between two tile positions
def heuristic(X, Y):
    x = threeDimMap[X]
    y = threeDimMap[Y]
    distance = math.sqrt(sum([(a - b) ** 2 for a, b in zip(x, y)]))
    return 2*distance


def main(algorithm, file):


	def Astar_Search():
		
		def Astar(n, k, MaxQueLen):
			#initialize open and closed lists
			openList=set()
			closedList=set()

			#add start state to openList
			openList.add(gs)
		
			while(len(openList)!=0):
				#find state with min f val
				minVal=10000000000
				for state in openList:
					if(state.getCurrentHeuristic() + state.g < minVal):
						minVal = state.getCurrentHeuristic() + state.g
						q = state
				print (minVal)
			
				#q.g=q.g+(2*(math.pi)/12)
			
				#create successors
				gs1 = copy.deepcopy(q)
				gs2 = copy.deepcopy(q)
				gs3 = copy.deepcopy(q)
				gs4 = copy.deepcopy(q)
				gs5 = copy.deepcopy(q)
				gs6 = copy.deepcopy(q)
			
				gs1.RC_axis0_180()
				gs1.path.append("RC_axis0_180")
				gs2.RCC_axis0_180()
				gs2.path.append("RCC_axis0_180")
				gs3.RC_axis90_270()
				gs3.path.append("RC_axis90_270")
				gs4.RCC_axis90_270()
				gs4.path.append("RCC_axis90_270")
				gs5.RC_equator()
				gs5.path.append("RC_equator")
				gs6.RCC_equator()
				gs6.path.append("RCC_equator")
			
				#for each succesor, if successor in either list and f val in listed item is greater, skip this successor.
				#else add it to openList
				for state in [gs1, gs2, gs3, gs4, gs5, gs6]:
					if(state.isSolved()):
						return (state, k, MaxQueLen)
				
					state.g = q.g + (2*(math.pi)/12)
					f = state.g + state.getCurrentHeuristic()
				
					flag = True 
				
					if(state.g > n):
						flag=False
					
					for stateX in closedList:
						if (stateX.axis0_180 == state.axis0_180) & (stateX.axis90_270 == state.axis90_270) & (stateX.equator == state.equator):
							#if stateX.getCurrentHeuristic() + stateX.g < f:
							flag = False
							#print("YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY")
				
					for stateX in openList:
						if (stateX.axis0_180 == state.axis0_180) & (stateX.axis90_270 == state.axis90_270) & (stateX.equator == state.equator):
							if state.g < stateX.g:
								#print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
								openList.remove(stateX)
								openList.add(state)
							flag = False
				
					if(flag==True):
						k=k+1
						openList.add(state)
			
				#remove q from openList
			
				if(len(openList)>MaxQueLen):
					MaxQueLen = len(openList)
			
				openList.remove(q)
				closedList.add(q)
		
		goal, k, MaxQueLen = Astar(100, 0, 0)
		
		print("Number of states expanded = " + str(k) + "\n")
		print("Max size of Queue = " + str(MaxQueLen) + "\n")
		print("Final Path Length = " + str(len(goal.path)))
		print("Final path = " + str(goal.path))
        

	def RBFS_Search():
		cueue = Queue(maxsize=0)
		maxCueLen = 0
		k = 0
		def RBFS(q, fval, cueue):
			if(q.isSolved()):
				return q
			
			k=k+1
		
			gs1 = copy.deepcopy(q)
			gs2 = copy.deepcopy(q)
			gs3 = copy.deepcopy(q)
			gs4 = copy.deepcopy(q)
			gs5 = copy.deepcopy(q)
			gs6 = copy.deepcopy(q)
		
			gs1.RC_axis0_180()
			gs1.path.append("RC_axis0_180")
			gs2.RCC_axis0_180()
			gs2.path.append("RCC_axis0_180")
			gs3.RC_axis90_270()
			gs3.path.append("RC_axis90_270")
			gs4.RCC_axis90_270()
			gs4.path.append("RCC_axis90_270")
			gs5.RC_equator()
			gs5.path.append("RC_equator")
			gs6.RCC_equator()
			gs6.path.append("RCC_equator")
		
			cueue.put(gs1)
			cueue.put(gs2)
			cueue.put(gs3)
			cueue.put(gs4)
			cueue.put(gs5)
			cueue.put(gs6)
			
			if(cueue.qsize()>maxCueLen):
				maxCueLen = len(cue)
		
			minF = 10000000
		
			sentinel = object()
			for state in iter(cueue.get, sentinel):     
				state.g = state.g + (2*(math.pi)/12)
				fval = max(state.g + state.getCurrentHeuristic(), fval)
				if(fval < minF):
					minF = fval
					stateX = state
			return RBFS(stateX, fval, cueue)
		
		goal=RBFS(gs, 0, cueue)
		
		print("Number of states expanded = " + k + "\n")
		print("Max size of Queue = " + maxCueLen + "\n")
		print("Final Path Length = " + str(len(goal.path)))
		print("Final path = " + str(goal.path))
		

	def BFS_Search():
		cueue = Queue(maxsize=0)
		cueue.put(gs)
		maxCueLen = 0
		k=0
		
		def BFS():
			while(cueue.empty()!=True):
				q = cueue.get()
				
				k=k+1
			
				if(q.isSolved()):
					return q
			
				gs1 = copy.deepcopy(q)
				gs2 = copy.deepcopy(q)
				gs3 = copy.deepcopy(q)
				gs4 = copy.deepcopy(q)
				gs5 = copy.deepcopy(q)
				gs6 = copy.deepcopy(q)
		
				gs1.RC_axis0_180()
				gs1.path.append("RC_axis0_180")
				gs2.RCC_axis0_180()
				gs2.path.append("RCC_axis0_180")
				gs3.RC_axis90_270()
				gs3.path.append("RC_axis90_270")
				gs4.RCC_axis90_270()
				gs4.path.append("RCC_axis90_270")
				gs5.RC_equator()
				gs5.path.append("RC_equator")
				gs6.RCC_equator()
				gs6.path.append("RCC_equator")
		
				cueue.put(gs1)
				cueue.put(gs2)
				cueue.put(gs3)
				cueue.put(gs4)
				cueue.put(gs5)
				cueue.put(gs6)
				
				if(cueue.qsize()>maxCueLen):
					maxCueLen = len(cue)
		
		goal = BFS()
		
		print("Number of states expanded = " + k + "\n")
		print("Max size of Queue = " + maxCueLen + "\n")
		print("Final Path Length = " + str(len(goal.path)))
		print("Final path = " + str(goal.path))


	
	axis0_180 = {}
	axis90_270 = {}
	equator = {}

	def populateAxes(tileNums):
		actualPos = (int(tileNums[0]), int(tileNums[1]))
		truePos = (int(tileNums[2]), int(tileNums[3]))
                
		#populate equator
		if (actualPos[0] == 90):
			equator[actualPos] = truePos
        
		#populate axis0-180
		if (actualPos[1] == 0):
			axis0_180[actualPos] = truePos
		if (actualPos[1] == 180):
			axis0_180[actualPos] = truePos
                
		#populate axis90-270
		if(actualPos[1] == 90):
			axis90_270[actualPos] = truePos
		if(actualPos[1] == 270):
			axis90_270[actualPos] = truePos
		if(actualPos[0] == 0 & actualPos[1] == 0):
			axis90_270[actualPos] = truePos
		if(actualPos[0] == 180 & actualPos[1] == 180):
			axis90_270[actualPos] = truePos
    
	def readState(filename):
		file1 = open(filename,"r")
		lines = file1.readlines()
		#print (lines[1:31])
		#print("\n")
        
		for line in lines[1:31]:
			tileNums = re.findall(r'\d+', line)
			#print(tileNums)
			if(len(tileNums)>4):
				#print(tileNums[2:6])
				populateAxes(tileNums[2:6])
			else:
				#print(tileNums)
				populateAxes(tileNums)

	readState(file)
	gs = GlobeState(axis0_180, axis90_270, equator, 0, [])
	
	if (algorithm == "BFS"):
		BFS_Search()
	if (algorithm == "RBFS"):
		RBFS_Search()
	if (algorithm == "Astar"):
		Astar_Search()
	


if __name__ == "__main__":
	 algorithm = str(sys.argv[1])
	 file = str(sys.argv[2])
	 
	 main(algorithm, file)
import numpy as np
import scipy as sp
import random 
import matplotlib.pyplot as plt
'''
    Inputs of the class: 
    road length, traffic density, maximum velocity, 
    the probability of slowing down, the probability of changing lanes
'''
class TrafficSimulation_3lane:
    
    def __init__(self, road_length = 100, density = 0.2, max_velocity = 5, slow_prob = 0.3, lane_prob = 0.8):
        self.road_length = road_length
        self.density = density
        self.max_velocity = max_velocity
        self.slow_prob = slow_prob
        self.lane_prob = lane_prob
        self.flow_count = 0
        
        #Set up 3 empty lanes
        self.state1 = -sp.ones(100, dtype=int)
        self.state2 = -sp.ones(100, dtype=int)
        self.state3 = -sp.ones(100, dtype=int)
    
        #Populate cars on the lanes according to initial probabilities
        for i in range(300):
            if random.random() <= self.density:
                if i<100:
                    self.state1[i] = random.randint(0,5)
                elif 100<i<200:
                    self.state2[i-100] = random.randint(0,5)
                else:
                    self.state3[i-200] = random.randint(0,5)            
        
    def update(self): 
        m = self.max_velocity
        
        next_state1 = -sp.ones(100, dtype=int)
        next_state2 = -sp.ones(100, dtype=int)
        next_state3 = -sp.ones(100, dtype=int)

        #find all the cars' indice
        current_index1 = np.argwhere(self.state1!=-1)
        current_index2 = np.argwhere(self.state2!=-1)
        current_index3 = np.argwhere(self.state3!=-1)
        update_index1 = current_index1
        update_index2 = current_index2
        update_index3 = current_index3        
        
        '''Determine if cars on lane #1 will change lanes '''
        for i in range(len(current_index1)):
            c = current_index1[i] #current cell index
            n1 = current_index1[(i+1) % len(current_index1)] #next cell index
            v = self.state1[c]
            
            #Symmetric model: a car only changes lanes when there's someone in front of it where gap < speed
            #Note that the gap equals distance - 
            if (n1-c)%100-1 >= v + 1: 
                break #doesn't change lanes
                
            #check if there's a car at the same index next lane, 
            if self.state2[c] >= 0 or (n1-c)%100 > v:
                break #doesn't change lanes

            #look back on the other lane
            for dx in range(1, m): 
                if self.state2[(c - dx)%100] >= 0: #there is a car behind on the other lane             
                    loback = False
                else: 
                    loback = True

            #find the car ahead in the other lane
            for dx in range(1, m):
                if self.state2[(c + dx)%100] >= 0: 
                    lo = dx - 1 #the neighborhood ahead has a car
                    break
                else:
                    lo = m - 1 #the neighborhood ahead is empty

            #change lanes
            if loback == True and lo >= v and random.random()<= self.lane_prob:
                
                #find the current cell index in the list of car indices
                c_index = np.argwhere(update_index1 == c)
                #delete the index from car indices the original lane  
                new_index1 = np.delete(update_index1, c_index)
                #because numpy array is immutable
                update_index1 = new_index1

                #add the index to the destination lane
                update_index2 = update_index2.flatten()
                #find the correct index to add to the sorted list of car indices
                insertindex = np.searchsorted(update_index2, c[0])
                #insert the index in the correct order
                update_index2 = np.insert(update_index2, insertindex, c)

        '''Determine if cars on lane #3 will change lanes '''
        for i in range(len(current_index3)):
            c = current_index3[i] #current cell index
            n3 = current_index3[(i+1) % len(current_index3)] #next cell index
            v = self.state3[c]
            
            #Symmetric model: a car only changes lanes when there's someone in front of it where gap < speed
            #Note that the gap equals distance - 
            if (n3-c)%100-1 >= v + 1: 
                break #doesn't change lanes
                
            #check if there's a car at the same index next lane, 
            if self.state3[c] >= 0 or (n3-c)%100 > v:
                break #doesn't change lanes

            #look back on the other lane
            for dx in range(1, m): 
                if self.state2[(c - dx)%100] >= 0: #there is a car behind on the other lane             
                    loback = False
                else: 
                    loback = True

            #find the car ahead in the other lane
            for dx in range(1, m):
                if self.state2[(c + dx)%100] >= 0: 
                    lo = dx - 1 #the neighborhood ahead has a car
                    break
                else:
                    lo = m - 1 #the neighborhood ahead is empty

            #change lanes
            if loback == True and lo >= v and random.random()<= self.lane_prob:
                
                #find the current cell index in the list of car indices
                c_index = np.argwhere(update_index3 == c)
                #delete the index from car indices the original lane  
                new_index3 = np.delete(update_index3, c_index)
                #because numpy array is immutable
                update_index3 = new_index3

                #add the index to the destination lane
                update_index2 = update_index2.flatten()
                #find the correct index to add to the sorted list of car indices
                insertindex = np.searchsorted(update_index2, c[0])
                #insert the index in the correct order
                update_index2 = np.insert(update_index2, insertindex, c)


        '''Determine if cars on lane 2 will change lanes '''
        for i in range(len(current_index2)):
            c = current_index2[i] #current cell index
            n2 = current_index2[(i+1) % len(current_index2)] #next cell index
            v = self.state2[c]
            
            #check the space ahead on the same lane
            if (n2-c)%100 >= v + 1:
                break
            
            #check the cell next to it on lane 1
            if self.state1[c] >= 0:
                break
            #check the cell next to it on lane 3
            if self.state3[c] >= 0:
                break
            
            #look back on lane 1
            for dx in range(1, m): 
                if self.state1[(c - dx)%100] >= 0: #there is a car behind on the other lane             
                    loback = False
                else: 
                    loback = True
                   
            #look back on lane 3
            for dx in range(1, m): 
                if self.state3[(c - dx)%100] >= 0: #there is a car behind on the other lane             
                    loback = False
                else: 
                    loback = True
                    
            #look ahead on lane 1
            for dx in range(1, m):
                if self.state1[(c + dx)%100] >= 0: 
                    lo = dx - 1
                    break
                else:
                    lo = m - 1
            
            #look ahead on lane 3
            for dx in range(1, m):
                if self.state3[(c + dx)%100] >= 0: 
                    lo = dx - 1
                    break
                else:
                    lo = m - 1   
            
            #change lanes
            if loback == True and lo >= v and random.random()<= self.lane_prob:
                
                #find the current cell index in the list of car indices
                c_index = np.argwhere(update_index2 == c)
                #delete the index from car indices the original lane
                new_index2 = np.delete(update_index2, c_index)
                #because numpy array is immutable
                update_index2 = new_index2
                
                #add the index to the destination lane 1
                update_index1 = update_index1.flatten()
                #find the correct index to add to the sorted list of car indices
                insertindex = np.searchsorted(update_index1, c[0])
                #insert the index in the correct order
                update_index1 = np.insert(update_index1, insertindex, c)
                
                
                #add the index to the destination lane 3
                update_index3 = update_index3.flatten()
                #find the correct index to add to the sorted list of car indices
                insertindex = np.searchsorted(update_index3, c[0])
                #insert the index in the correct order
                update_index3 = np.insert(update_index3, insertindex, c)
                
        
        '''update the car movements on both lane'''
        ###LANE 1###
        for i in range(len(update_index1)):
        #the index of interest now and the next one
            c = update_index1[i]
            n = update_index1[(i+1) % len(update_index1)]
            v = self.state1[c]
            
            #accelearation
            if v < m and (n-c)%100 > v:
                v = v + 1
                 
            #slowing down
            if v > 0 and (n-c)%100 <= v :
                v = (n-c)%100 -1
                 
            #randomization            
            if v > 0 and random.random() <= self.slow_prob:
                v = v-1
            
            #update speed and movement 
            next_state1[(c + v) % 100] = v
            
            if c + v >= 100:
                self.flow_count += 1 #update flow count
                
        ###LANE 3###
        for i in range(len(update_index3)):
        #the index of interest now and the next one
            c = update_index3[i]
            n = update_index3[(i+1) % len(update_index3)]
            v = self.state1[c]
            
            #accelearation
            if v < m and (n-c)%100 > v:
                v = v + 1
                 
            #slowing down
            if v > 0 and (n-c)%100 <= v :
                v = (n-c)%100 -1
                 
            #randomization            
            if v > 0 and random.random() <= self.slow_prob:
                v = v-1
            
            #update speed and movement 
            next_state3[(c + v) % 100] = v
            
            if c + v >= 100:
                self.flow_count += 1 #update flow count
         
        ###LANE 2###
        for i in range(len(update_index2)):
        #the index of interest now and the next one
            c = update_index2[i]
            n = update_index2[(i+1) % len(update_index2)]
            v = self.state2[c]
            #print c,n,v 
            
            #accelearation
            if v < m and (n-c)%100 > v:
                v = v + 1
                 
            #slowing down
            if v > 0 and (n-c)%100 <= v :
                v = (n-c)%100 - 1
                 
            #randomization            
            if v > 0 and random.random() <= self.slow_prob:
                v = v - 1
             
            #update speed and movement 
            next_state2[(c + v) % 100] = v
        
            if c + v >= 100:
                self.flow_count += 1 #update flow count
       
        #update the states of both lane
        self.state1 = next_state1
        self.state2 = next_state2
        self.state3 = next_state3
        
    def display(self):
        print(''.join('.' if x == -1 else str(x) for x in self.state1)),"____",
        print(''.join('.' if x == -1 else str(x) for x in self.state2)),"____",
        print(''.join('.' if x == -1 else str(x) for x in self.state3))
        print
    
    
random.seed(100)

#set up the simulation 
T3 = TrafficSimulation_3lane(density = 0.15)
#time steps
t=50

for i in range(t):
    T3.update()
    T3.display()
    
    
random.seed(100)
#calculate the flow rate for the three-lane model

flow_rate_3 = []
densities = []
time_step = 200

for i in range(1,20): #loop through the densities
    p = 0.05*i
    densities.append(p) #density
    p_all_flow_rates = [] #store the flow rates of each simulation for one density
   
    for j in range(20): #repeat simulation each density for 20 times
        T3 = TrafficSimulation_3lane(density = p) #simulation
        for i in range(time_step): #time step = 200
            T3.update()
        p_all_flow_rates.append(T3.flow_count/float(time_step))
    
    #average flow rate for one density
    flow_rate_3.append(sum(p_all_flow_rates)/20.0/3.0)

plt.plot(densities,flow_rate_3)
plt.title("Three-lane Traffic Simulation")
plt.xlabel("Density")
plt.ylabel("Flow rate")  

print("For the two-lane model, the max flow rate is:", max(flow_rate_3))
print("and it's reached at the density of:", 0.02*flow_rate_3.index(max(flow_rate_3)))
 
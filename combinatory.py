import numpy as np
import math
from dubin import dubins_path_planner, dubin_path_length, get_current_pose, sample_path
from utils1 import dijkstra, get_distance_point_2_edge, get_angle, get_abs_angle, rotate, point_in_polygon, test_edge_intersect, sample_point_on_edge
import pyvoronoi
import matplotlib.pyplot as plt
import copy


class dubin_interpolation:      
    
    def __init__(self,value,start_theta,end_theta,path):
        self.min_value_list = value
        self.start_theta_list = start_theta
        self.end_theta_list = end_theta
        self.path_list = path

class Dubin_path_segment():

    def __init__(self,direction,length):

        self.dir = direction
        self.length = length

class Node:

    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.child = []
        self.min_dis = []

    def __eq__(self, ob):
        return ((self.x == ob.x) and (self.y == ob.y))
    
    def __ne__(self, ob):
        return ((self.x != ob.x) or (self.y != ob.y))
    
    def addEdge(self,node):
        self.child.append(node)

    def addMinDis(self,dis):
        self.min_dis.append(dis)

class Node2:

    def __init__(self,x,y,theta,time):
        self.x = x
        self.y = y
        self.theta = theta % (2*np.pi)
        self.time = time

    def __eq__(self, ob):
        return ((round(self.x -ob.x,3) == 0) and (round(self.y -ob.y,3) == 0) and (round(self.theta -ob.theta,3) == 0) and (round(self.time -ob.time,3) == 0))
    

class Simple_Node2:
    def __init__(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta % (2*np.pi)

    def __eq__(self, ob):
        return ((round(self.x -ob.x,3) == 0) and (round(self.y -ob.y,3) == 0) and (round(self.theta -ob.theta,3) == 0))

class Edge():
    
    def __init__(self, vertices):
        
        self.vertices = vertices
        self.edge  = vertices[0] - vertices[1]
        if self.edge[0] == self.edge[1] and self.edge[1] == 0:
            self.normal = None
        else:
            if self.edge[1] != 0:
                a = 1/(1+pow(self.edge[0]/self.edge[1],2))
                a = pow(a,0.5)
                b = -a*self.edge[0]/self.edge[1]
            else:
                b = 1/(1+pow(self.edge[1]/self.edge[0],2))
                b = pow(b,0.5)
                a = -b*self.edge[1]/self.edge[0]
            self.normal = np.array([a,b])

    def __eq__(self, ob):
        return (((self.vertices[0][0] == ob.vertices[0][0] and self.vertices[0][1] == ob.vertices[0][1]) 
                and (self.vertices[1][0] == ob.vertices[1][0] and self.vertices[1][1] == ob.vertices[1][1])) 
        or ((self.vertices[0][0] == ob.vertices[1][0] and self.vertices[0][1] == ob.vertices[1][1])
             and (self.vertices[1][0] == ob.vertices[0][0] and self.vertices[1][1] == ob.vertices[0][1])))


class Obstacle():
         
    def __init__(self, vertices):
        
        self.edge = []
        closed_list = []
        # Initialize the first two edges
        
        closed_list.append(0)
        max_angle = 0
        for i in range(1,len(vertices)):
            vector1 = vertices[0]-vertices[i]
            for k in range(1,len(vertices)):
                vector2 = vertices[0]-vertices[k]
                angle = get_angle(vector1,vector2)
                if angle > math.pi:
                    angle = 2*math.pi - angle
                if angle > max_angle:
                    max_angle  = angle
                    index1 = i
                    index2 = k

        edge1  = Edge([vertices[0],vertices[index1]])
        edge2  = Edge([vertices[index2],vertices[0]])
        self.edge.append(edge1)
        

        # Connect other edges

        current_index = index1
        current_edge = edge1
        

        while current_index != index2:
            if not current_index in closed_list:
                max_angle = 0
                for i in range(0,len(vertices)):
                    if not i in closed_list and i!= current_index:
                        vector = vertices[current_index]-vertices[i]
                        angle = get_angle(vector, -current_edge.edge)
                        if angle > math.pi:
                            angle = 2*math.pi - angle
                        if angle > max_angle:
                            max_angle = angle
                            next_index = i

                next_edge = Edge([vertices[current_index],vertices[next_index]])
                self.edge.append(next_edge)
                closed_list.append(current_index)
                # Update for next iteration

                current_edge = next_edge
                current_index = next_index

        self.edge.append(edge2)

        self.vertices = vertices

class Map:
    
    def __init__(self, map_border_vertices, obstacles_list,gates):

        self.obstacles = obstacles_list
        self.offset_obstacles = []
        
        self.map = Obstacle(map_border_vertices)

        self.gates = gates
        
        self.limo_vertices = np.array(
            [np.array([0.3,0.15]),
            np.array([0.3,-0.15]),
            np.array([-0.3,-0.15]),
            np.array([-0.3,0.15])]
        )
        self.limo_vertices = np.transpose(self.limo_vertices)
        self.limo_radius = 0.335
        self.limo_turning_rad = 0.4
        self.node_list = []

    def plot_map(self):
        
        # Plot map border
        for edge in self.map.edge:
            x = [edge.vertices[0][0],edge.vertices[1][0]]
            y = [edge.vertices[0][1],edge.vertices[1][1]]
            plt.plot(x,y)
        
        # Plot obstales
        for obstacle in self.obstacles:
            for edge in obstacle.edge:
                x = [edge.vertices[0][0],edge.vertices[1][0]]
                y = [edge.vertices[0][1],edge.vertices[1][1]]
                plt.plot(x,y)

    def plot_offset_map(self):
        
        # Plot map border
        for edge in self.offset_map.edge:
            x = [edge.vertices[0][0],edge.vertices[1][0]]
            y = [edge.vertices[0][1],edge.vertices[1][1]]
            plt.plot(x,y)
        
        # Plot obstales
        for obstacle in self.offset_obstacles:
            for edge in obstacle.edge:
                x = [edge.vertices[0][0],edge.vertices[1][0]]
                y = [edge.vertices[0][1],edge.vertices[1][1]]
                plt.plot(x,y)
        
        # Plot roadmap
        # for edge in self.voronoi_edges:
        #     x = [edge.vertices[0][0],edge.vertices[1][0]]
        #     y = [edge.vertices[0][1],edge.vertices[1][1]]
        #     plt.plot(x,y, marker = 'o')

    def plot_path(self,sample_path):
        for i in range(len(sample_path)-1):
            x = [sample_path[i][0],sample_path[i+1][0]]
            y = [sample_path[i][1],sample_path[i+1][1]]
            plt.plot(x,y)

    def plot_rotated_map(self):
        
        # Plot map border
        for edge in self.rotated_map.edge:
            x = [edge.vertices[0][0],edge.vertices[1][0]]
            y = [edge.vertices[0][1],edge.vertices[1][1]]
            plt.plot(x,y)
        
        # Plot obstales
        for obstacle in self.rotated_obstacles:
            for edge in obstacle.edge:
                x = [edge.vertices[0][0],edge.vertices[1][0]]
                y = [edge.vertices[0][1],edge.vertices[1][1]]
                plt.plot(x,y)

    def approximate_cell_decomposition(self,resolution):
        
        self.resolution = resolution
        # Rotate the map

        theta = np.pi/2 -self.gates[2]
        vertices_list = []
        for vertice in self.offset_map.vertices:
            rotated_vertice = rotate(theta,vertice)
            vertices_list.append(rotated_vertice)
        
        self.rotated_map = Obstacle(np.array(vertices_list))

        self.rotated_obstacles = []
        for obstacle in self.offset_obstacles:
            vertices_list = []
            for vertice in obstacle.vertices:
                rotated_vertice = rotate(theta,vertice)
                vertices_list.append(rotated_vertice)
            ob = Obstacle(np.array(vertices_list))
            self.rotated_obstacles.append(ob)

        [x,y] = rotate(theta,[self.gates[0],self.gates[1]])
        rotated_gate = [x,y,np.pi/2]

        # End rotate map

        x_min = 10000
        x_max = -10000
        y_min = 10000
        y_max = -10000
        for vertice in self.rotated_map.vertices:
            
            if x_min > vertice[0]:
                x_min = vertice[0]
                
            if x_max < vertice[0]:
                x_max = vertice[0]

            if y_min > vertice[1]:
                y_min = vertice[1]

            if y_max < vertice[1]:
                y_max = vertice[1]
        
        x = rotated_gate[0] - resolution/2
        y = rotated_gate[1]
        x_min = round(x - ((x-x_min)-((x-x_min)%resolution) + resolution),4)
        x_max = round(x + ((x_max-x)-((x_max-x)%resolution) + resolution),4)
        y_min = round(y - ((y-y_min)-((y-y_min)%resolution) + resolution),4)
        y_max = round(y + ((y_max-y)-((y_max-y)%resolution) + resolution),4)
        
        horizontal_cell_number = int(round((x_max - x_min)/resolution))
        vertical_cell_number = int(round((y_max - y_min)/resolution)) 
        cell_array = np.zeros((vertical_cell_number,horizontal_cell_number))

        for i in range(vertical_cell_number):
            y1 = y_min + i*resolution
            y2 = y1 + resolution
            for k in range(horizontal_cell_number):
                sub_cell = []
                x1 = x_min + k*resolution
                x2 = x1 + resolution

                sub_cell.append(Obstacle([np.array([x1,y2]), np.array([x1+0.5*resolution,y2]),
                                      np.array([x1,y1+0.5*resolution]),np.array([x1+0.5*resolution,y1+0.5*resolution])]))
                
                sub_cell.append(Obstacle([np.array([x1+0.5*resolution,y2]), np.array([x2,y2]),
                        np.array([x1+0.5*resolution,y1+0.5*resolution]),np.array([x2,y1+0.5*resolution])]))
                
                sub_cell.append(Obstacle([np.array([x2,y1+0.5*resolution]), np.array([x2,y1]),
                        np.array([x1+0.5*resolution,y1]),np.array([x1+0.5*resolution,y1+0.5*resolution])]))
                
                sub_cell.append(Obstacle([np.array([x1,y1+0.5*resolution]), np.array([x1,y1]),
                        np.array([x1+0.5*resolution,y1]),np.array([x1+0.5*resolution,y1+0.5*resolution])]))
                
                for j in range(len(sub_cell)):
                    cell_empty = True
                    for edge in sub_cell[j].edge:
                        for ed in self.rotated_map.edge:
                            if test_edge_intersect(ed,edge):
                                cell_empty = False
                                cell_array[vertical_cell_number-1-i][k] += 2**j
                                break         
                        if not cell_empty:
                            break
                    if not cell_empty:
                        continue

                    for obstacle in self.rotated_obstacles:
                        for edge in sub_cell[j].edge:
                            for ed in obstacle.edge:
                                if test_edge_intersect(ed,edge):
                                    cell_empty = False
                                    cell_array[vertical_cell_number-1-i][k] += 2**j
                                    break
                            if not cell_empty:
                                break
                        if not cell_empty:
                            break    
        self.x_min = x_min
        self.y_min = y_min
        self.cell_array = cell_array
        self.horizontal_cell_number = horizontal_cell_number
        self.vertical_cell_number = vertical_cell_number
        
        
    
    def get_cell_index(self,pose):
        horizontal_index = int(round((pose[0] - self.x_min)/self.resolution,2))
        vertical_index = int(round((pose[1]- self.y_min)/self.resolution,2))
        if round(pose[2],3) == round(np.pi/2,3):
            x_index = horizontal_index
            y_index = self.vertical_cell_number-1-vertical_index
        elif round(pose[2],3) == round(3*np.pi/2,3):
            x_index = horizontal_index
            y_index = self.vertical_cell_number - vertical_index
        elif round(pose[2],3) == round(np.pi,3):
            x_index = horizontal_index-1
            y_index = self.vertical_cell_number - 1 - vertical_index
        elif round(pose[2],3) == round(2*np.pi,3) or round(pose[2],3) == 0:
            x_index = horizontal_index
            y_index = self.vertical_cell_number - 1- vertical_index

        return x_index, y_index
    
    
    def get_pose_at_time(self,pose,path,time):
        
        i = 0
        current_pose = pose
        while  time >= path[i].length:
            current_pose = get_current_pose(path[i], current_pose, self.limo_turning_rad)
            time -= path[i].length
            i += 1
            if i >= len(path):
                break
        if i == len(path):
            return [10000,10000,0],-1,1
        else:
            seg_left = Dubin_path_segment(path[i].dir,time)
            current_pose = get_current_pose(seg_left,current_pose,self.limo_turning_rad)
            return current_pose,i,path[i].length-time
        
    def get_action_during_time(self,path,time1,time2):

        current_pose,last_index, left_over_length = self.get_pose_at_time([0,0,0],path,time1)
        n_path = []
        if last_index != -1:
            n_path.append(Dubin_path_segment(path[last_index].dir,left_over_length))
            i = last_index + 1
            if i < len(path):
                l = time2 - time1
                while  l >= path[i].length and i < len(path):
                    l -= path[i].length
                    n_path.append(path[i])
                    i += 1
                    if i >= len(path):
                        break
                if i < len(path):
                    p = Dubin_path_segment(path[i].dir,l)
                    n_path.append(p)
            return n_path
        else:
            return None


    def action_allow(self,x,y,theta,action):
        
        # action == 0 STRAIGHT
        # action == 1 RIGHT
        # action == 2 LEFT


        x_index, y_index= self.get_cell_index([x,y,theta])
        if y_index >= len(self.cell_array) or x_index >= len(self.cell_array[0]):
            return False
        # Convert cell array to binary
        a = self.cell_array[y_index][x_index]
        b = []
        for p in range(3,-1,-1):
            b.insert(0,round((a - (a%(2**p)))/2**p))
            a = a - b[0]*2**p
        # End convert
        if (round(theta -np.pi/2,3) == 0 ) and ((action == 0 and self.cell_array[y_index][x_index] >= 1)
                                or (action == 1 and b[2] == 1)
                                or (action == 2 and b[3] == 1) ):
            return False
        elif (round(theta -3*np.pi/2,3) == 0) and ((action == 0 and self.cell_array[y_index][x_index] >= 1)
                                or (action == 1 and b[0] == 1)
                                or (action == 2 and b[1] == 1) ):
            return False
        elif (round(theta -2*np.pi,3) == 0 or round(theta,3) == 0) and ((action == 0 and self.cell_array[y_index][x_index] >= 1)
                                or (action == 1 and b[3] == 1)
                                or (action == 2 and b[0] == 1) ):
            return False
        elif (round(theta - np.pi,3) == 0) and ((action == 0 and self.cell_array[y_index][x_index] >= 1)
                                or (action == 1 and b[1] == 1)
                                or (action == 2 and b[2] == 1) ):
            return False
        else:  
            return True
    
    def limo1_path_exist(self, start_pose, end_pose, x_min,y_min, cell_array,resolution, vertical_cell_number):
        dubin_path_straight = dubins_path_planner([0,0,np.pi/2],[0,resolution,np.pi/2],self.limo_turning_rad)
        dubin_path_straight.sort(key = dubin_path_length)
        dubin_path_left = dubins_path_planner([0,0,np.pi/2],[-resolution/2,resolution/2,np.pi],self.limo_turning_rad)
        dubin_path_left.sort(key = dubin_path_length)
        dubin_path_right = dubins_path_planner([0,0,np.pi/2],[resolution/2,resolution/2,0],self.limo_turning_rad)
        dubin_path_right.sort(key = dubin_path_length)
        dubin_path_list = [dubin_path_straight[0],dubin_path_right[0],dubin_path_left[0]]
        
        closed_list = []
        final_cost_list = []
        final_path_list = []
        open_list = []
        cost_list = []
        path_list = []
        node = Simple_Node2(start_pose[0],start_pose[1],start_pose[2])
        closed_list.append(node)
        final_cost_list.append(0)
        final_path_list.append([])
    
        for k in range(len(dubin_path_list)):
            dubin_path = dubin_path_list[k]
            x,y,theta = node.x, node.y, node.theta
            length = 0
            if self.action_allow(x,y,theta,k):
                for seg in dubin_path:
                    [x,y,theta] = get_current_pose(seg,[x,y,theta],self.limo_turning_rad)
                    length += seg.length

                length = length + final_cost_list[-1]
                path = copy.deepcopy(final_path_list[-1])
                new_node = Simple_Node2(x,y,theta)
                path.append(dubin_path)
                open_list.append(new_node)
                cost_list.append(length)
                path_list.append(path)
        
        if not open_list:
            return None, False
        while open_list:
            res = np.where(cost_list == np.min(cost_list))[0]
            node = open_list.pop(res[0])
            cost = cost_list.pop(res[0])
            path = path_list.pop(res[0])
            closed_list.append(node)
            final_cost_list.append(cost)
            final_path_list.append(path)
            if (round(node.x,4) == round(end_pose[0],4)) and (round(node.y,4) == round(end_pose[1],4)) and (round(node.theta,4) == round(end_pose[2],4)):
                    return path, True

            for k in range(len(dubin_path_list)):
                dubin_path = dubin_path_list[k]
                length = 0
                x,y,theta = node.x, node.y, node.theta
                if self.action_allow(x,y,theta,k):
                    for seg in dubin_path:
                        [x,y,theta] = get_current_pose(seg,[x,y,theta],self.limo_turning_rad)
                        length += seg.length
                    new_node = Simple_Node2(x,y,theta)
                    if not new_node in closed_list:
                        if not new_node in open_list:
                            path = copy.deepcopy(final_path_list[-1])                        
                            path.append(dubin_path)
                            open_list.append(new_node)
                            cost_list.append(length+ final_cost_list[-1])
                            path_list.append(path)
                        else:
                            i = open_list.index(new_node)
                            path = copy.deepcopy(final_path_list[-1])
                            path.append(dubin_path)
                            if length + final_cost_list[-1] < cost_list[i]:
                                cost_list[i] = length + final_cost_list[-1]
                                path_list[i] = path
        return None, False
    
    
    def A_star(self,limo0,limo0_path, start_pose, end_pose,x_min,y_min,cell_array,resolution, vertical_cell_number,path_reference):
        # Get limo0 path length
        limo0_path_length = 0
        for seg in limo0_path:
            limo0_path_length += seg.length
        # End get limo0 path length

        dubin_path_straight = dubins_path_planner([0,0,np.pi/2],[0,resolution,np.pi/2],self.limo_turning_rad)
        dubin_path_straight.sort(key = dubin_path_length)
        dubin_path_left = dubins_path_planner([0,0,np.pi/2],[-resolution/2,resolution/2,np.pi],self.limo_turning_rad)
        dubin_path_left.sort(key = dubin_path_length)
        dubin_path_right = dubins_path_planner([0,0,np.pi/2],[resolution/2,resolution/2,0],self.limo_turning_rad)
        dubin_path_right.sort(key = dubin_path_length)
        dubin_path_list = [dubin_path_straight[0],dubin_path_right[0],dubin_path_left[0]]

        closed_list = []
        final_cost_list = []
        final_path_list = []
        open_list = []
        cost_list = []
        path_list = []
        node = Node2(start_pose[0],start_pose[1],start_pose[2],0)
        closed_list.append(node)
        final_cost_list.append(((start_pose[0] -end_pose[0])**2 + (start_pose[1] -end_pose[1])**2)**0.5)
        final_path_list.append([])

        visited_list = []
        max_time_allow_list = []

        for k in range(len(dubin_path_list)):
            dubin_path = dubin_path_list[k]
            x,y,theta = node.x, node.y, node.theta
            length = 0
            if self.action_allow(x,y,theta,k):
                for seg in dubin_path:
                    [x,y,theta] = get_current_pose(seg,[x,y,theta],self.limo_turning_rad)
                    length += seg.length
                
                # Check collision with limo0
                current_limo0,last_index, left_over_length = self.get_pose_at_time(limo0,limo0_path,node.time)
                if ((node.x -current_limo0[0])**2 + (node.y -current_limo0[1])**2)**0.5  < 2*(self.limo_radius+length):
                    path = self.get_action_during_time(limo0_path,node.time,node.time+length)
                    if path:
                        if self.check_collision_2_limo_path([node.x,node.y,node.theta],dubin_path,current_limo0,path):
                            continue
                n = Simple_Node2(x,y,theta)
                if n in path_reference:
                    mul = 0.25
                else:
                    mul = 1
                l = (node.time+length+((x -end_pose[0])**2 + (y -end_pose[1])**2)**0.5)*mul
                path = copy.deepcopy(final_path_list[-1])
                new_node = Node2(x,y,theta,length + node.time)
                path.append(dubin_path)
                open_list.append(new_node)
                cost_list.append(l)
                path_list.append(path)

                if not n in visited_list:
                    visited_list.append(n)
                    current_limo0,last_index, left_over_length = self.get_pose_at_time(limo0,limo0_path,length + node.time)
                    if ((x-current_limo0[0])**2 + (y-current_limo0[1])**2)**0.5 > 2*(self.limo_radius+limo0_path_length -length - node.time):
                        max_time_allow_list.append([0,node.time + length,True])
                    else:
                        max_time_allow_list.append([node.time + length,0, False])
                else:
                    index = visited_list.index(n)
                    current_limo0,last_index, left_over_length = self.get_pose_at_time(limo0,limo0_path,length + node.time)
                    if max_time_allow_list[index][2]:
                        if length + node.time > max_time_allow_list[index][0] and length + node.time < max_time_allow_list[index][1]:
                            if ((x-current_limo0[0])**2 + (y-current_limo0[1])**2)**0.5 > 2*(self.limo_radius+limo0_path_length-length - node.time):
                                max_time_allow_list[index][1] = length + node.time
                            else:
                                max_time_allow_list[index][0] = length + node.time
                    else:
                        if ((x-current_limo0[0])**2 + (y-current_limo0[1])**2)**0.5 > 2*(self.limo_radius+limo0_path_length-length - node.time):
                            max_time_allow_list[index][1] = length + node.time
                            max_time_allow_list[index][2] = True
                        elif length + node.time > max_time_allow_list[index][0]:
                            max_time_allow_list[index][0] = length + node.time

        if not open_list:
            return False, None
        while open_list:
            res = np.where(cost_list == np.min(cost_list))[0]
            node = open_list.pop(res[0])
            cost = cost_list.pop(res[0])
            path = path_list.pop(res[0])
            print(node.x,node.y,node.theta)
            closed_list.append(node)
            final_cost_list.append(cost)
            final_path_list.append(path)
            if (round(node.x,4) == round(end_pose[0],4)) and (round(node.y,4) == round(end_pose[1],4)) and (round(node.theta,4) == round(end_pose[2],4)):
                    return True, path

            for k in range(len(dubin_path_list)):
                dubin_path = dubin_path_list[k]
                length = 0
                x,y,theta = node.x, node.y, node.theta
                if self.action_allow(x,y,theta,k):
                    for seg in dubin_path:
                        [x,y,theta] = get_current_pose(seg,[x,y,theta],self.limo_turning_rad)
                        length += seg.length
                    new_node = Node2(x,y,theta,node.time + length)
                    n = Simple_Node2(x,y,theta)
                    if not new_node in closed_list:
                        if not new_node in open_list:
                            if n in visited_list:
                                index = visited_list.index(n)
                                if (max_time_allow_list[index][2]) and (length + node.time > max_time_allow_list[index][1]):
                                    continue
                            # Check collision with limo0
                            current_limo0,last_index, left_over_length = self.get_pose_at_time(limo0,limo0_path,node.time)
                            if ((node.x -current_limo0[0])**2 + (node.y -current_limo0[1])**2)**0.5  < 2*(self.limo_radius+length):   
                                path = self.get_action_during_time(limo0_path,node.time,node.time+length)   
                                if path:                    
                                    if self.check_collision_2_limo_path([node.x,node.y,node.theta],dubin_path,current_limo0,path):
                                        continue
                            if n in path_reference:
                                mul = 0.25
                            else:
                                mul = 1
                            l = (length+node.time+((x -end_pose[0])**2 + (y -end_pose[1])**2)**0.5)*mul
                            path = copy.deepcopy(final_path_list[-1])                        
                            path.append(dubin_path)
                            open_list.append(new_node)
                            cost_list.append(l)
                            path_list.append(path)

                            if not n in visited_list:
                                visited_list.append(n)
                                current_limo0,last_index, left_over_length = self.get_pose_at_time(limo0,limo0_path,length + node.time)
                                if ((x-current_limo0[0])**2 + (y-current_limo0[1])**2)**0.5 > 2*(self.limo_radius+limo0_path_length -length - node.time):
                                    max_time_allow_list.append([0,node.time + length,True])
                                else:
                                    max_time_allow_list.append([node.time + length,0, False])
                            else:
                                index = visited_list.index(n)
                                current_limo0,last_index, left_over_length = self.get_pose_at_time(limo0,limo0_path,length + node.time)
                                if max_time_allow_list[index][2]:
                                    if length + node.time > max_time_allow_list[index][0] and length + node.time < max_time_allow_list[index][1]:
                                        if ((x-current_limo0[0])**2 + (y-current_limo0[1])**2)**0.5 > 2*(self.limo_radius+limo0_path_length-length - node.time):
                                            max_time_allow_list[index][1] = length + node.time
                                        else:
                                            max_time_allow_list[index][0] = length + node.time
                                else:
                                    if ((x-current_limo0[0])**2 + (y-current_limo0[1])**2)**0.5 > 2*(self.limo_radius+limo0_path_length-length - node.time):
                                        max_time_allow_list[index][1] = length + node.time
                                        max_time_allow_list[index][2] = True
                                    elif length + node.time > max_time_allow_list[index][0]:
                                        max_time_allow_list[index][0] = length + node.time
        

    def create_voronoi_roadmap(self):
    
        self.voronoi_edges = []
        self.voronoi_vertices = []
        self.dis_list = []
        pv = pyvoronoi.Pyvoronoi(100)

        for edge in self.offset_map.edge:
            segment = [[edge.vertices[0][0],edge.vertices[0][1]],[edge.vertices[1][0],edge.vertices[1][1]]]
            pv.AddSegment(segment)

        for obstacle in self.offset_obstacles:
            for edge in obstacle.edge:
                segment = [[edge.vertices[0][0],edge.vertices[0][1]],[edge.vertices[1][0],edge.vertices[1][1]]]
                pv.AddSegment(segment)
        
        pv.Construct()
        cells = pv.GetCells()
        edges = pv.GetEdges()
        vertices = pv.GetVertices()
        
        for cell in cells:
            if not cell.is_open: 
                for edge_index in cell.edges:
                    edge = edges[edge_index]
                    if edge.is_primary:
                        point1 = np.array([vertices[edge.start].X,vertices[edge.start].Y])
                        point2 = np.array([vertices[edge.end].X,vertices[edge.end].Y])
                        for obstacle in self.offset_obstacles:
                            check = point_in_polygon(point1,obstacle)
                            check = check or point_in_polygon(point2,obstacle)
                            if check:
                                break
                        if not check:
                            edge_list = []
                            if edge.is_linear:
                                edge_vertices = np.array([[vertices[edge.start].X,vertices[edge.start].Y],[vertices[edge.end].X,vertices[edge.end].Y]])
                                edge_vertices = np.round(edge_vertices,4)
                                ed = Edge(edge_vertices)
                                edge_list.append(ed)
                            else:
                                dis = ((vertices[edge.start].X-vertices[edge.end].X)**2 + (vertices[edge.start].Y-vertices[edge.end].Y)**2)**0.5/2
                                points = pv.DiscretizeCurvedEdge(edge_index,dis)
                                points_list = []
                                for p in points:
                                    points_list.append([p[0],p[1]])

                                for i in range(len(points_list)-1):
                                    vert = np.array([[points_list[i][0],points_list[i][1]],[points_list[i+1][0],points_list[i+1][1]]])
                                    vert = np.round(vert,4)
                                    ed = Edge(vert)
                                    edge_list.append(ed)

                            for ed in edge_list:
                                if (not ed in self.voronoi_edges):
                                    self.voronoi_edges.append(ed)
                                    node1 = Node(ed.vertices[0][0],ed.vertices[0][1])
                                    node2 = Node(ed.vertices[1][0],ed.vertices[1][1])
                                    if not node1 in self.voronoi_vertices:
                                        self.voronoi_vertices.append(node1)
                                        index1 = len(self.voronoi_vertices)-1
                                        if not node2 in self.voronoi_vertices:
                                            self.voronoi_vertices.append(node2)
                                            index2 = index1 +1
                                        else:
                                            index2 = self.voronoi_vertices.index(node2)
                                    else:
                                        index1 = self.voronoi_vertices.index(node1)
                                        if not node2 in self.voronoi_vertices:
                                            self.voronoi_vertices.append(node2)
                                            index2 = len(self.voronoi_vertices)-1
                                        else:
                                            index2 = self.voronoi_vertices.index(node2)
                                    
                                    dis = self.get_min_dis_to_edge(ed)
                                    self.voronoi_vertices[index1].addEdge(index2)
                                    self.voronoi_vertices[index2].addEdge(index1)
                                    self.voronoi_vertices[index1].addMinDis(dis)
                                    self.voronoi_vertices[index2].addMinDis(dis)

    def interpolation(self,voronoi_vertices,path_index, start_theta, end_theta, resolution):
    
        path = []
        min_dis_list = []
        for i  in range(len(path_index)):
            path.append([voronoi_vertices[path_index[i]].x,voronoi_vertices[path_index[i]].y])
            if i < len(path_index)-1:
                index = voronoi_vertices[path_index[i]].child.index(path_index[i+1])
                min_dis = voronoi_vertices[path_index[i]].min_dis[index]
                min_dis_list.append(min_dis)
        
        # Backward sweeping for node from goal to node before start node
        
        interpolation_list = []
        n = dubin_interpolation([0],[end_theta],[],[])
        interpolation_list.append(n)
        
        for index in range(len(path)-2,0,-1):
            theta = 0
            start_angle_list = []
            end_angle_list = []
            min_value_list = []
            min_path_list = []
            while theta < 2*np.pi:
                # Find end angle that result in min value correspond with theta as start angle
                end_theta_list = []
                value_list = []
                path_list = []
                for i in range(len(interpolation_list[0].start_theta_list)):
                    angle = interpolation_list[0].start_theta_list[i]
                    dubin_paths = dubins_path_planner([path[index][0],path[index][1],theta],[path[index+1][0],path[index+1][1],angle],self.limo_turning_rad)
                    end_theta_list.append(angle)

                    # Get dubin path with minimum length
                    min_dubin_length = 100000
                    for dubin_path in dubin_paths:
                        length = 0
                        have_straight_segment = False
                        for dubin_segment in dubin_path:
                            if dubin_segment.dir == 0:
                                have_straight_segment = True
                            length += dubin_segment.length
                        if have_straight_segment:
                            mul = 2
                        else:
                            mul = 4   
                        if min_dis_list[index] < mul*self.limo_turning_rad:                  
                            sample_pose = sample_path(dubin_path, [path[index][0],path[index][1],theta],self.limo_turning_rad/8,self.limo_turning_rad)
                            collision = False
                            for pose in sample_pose:

                                for obstacle in self.offset_obstacles:
                                    if point_in_polygon([pose[0],pose[1]],obstacle):
                                        collision = True
                                        break
                                
                                if collision:
                                    length += 10000
                                    break
                                else:
                                    if not point_in_polygon([pose[0],pose[1]],self.offset_map):
                                        length += 10000
                                        break

                        for dubin_segment in dubin_path:
                            length += dubin_segment.length
                        
                        if length < min_dubin_length:
                            min_dubin_length = length
                            min_path = dubin_path                   
                    # End get dubin path with minimum length
                    
                    min_value = min_dubin_length + interpolation_list[0].min_value_list[i]
                    value_list.append(min_value)
                    path_list.append(min_path)
                
                min_value = 1000000
                for i in range(len(value_list)):
                    if min_value > value_list[i]:
                        min_value = value_list[i]
                        min_index = i
                min_path = path_list[min_index]
                end_angle = end_theta_list[min_index]
                # End finding minimum value and end angle correspond with theta

                start_angle_list.append(theta)
                end_angle_list.append(end_angle)
                min_value_list.append(min_value)
                min_path_list.append(min_path)

                theta += resolution
            n = dubin_interpolation(min_value_list,start_angle_list,end_angle_list,min_path_list)
            interpolation_list.insert(0,n)       
        # Add interpolation for start node
        end_theta_list = []
        value_list = []
        path_list = []
        for i in range(len(interpolation_list[0].start_theta_list)):
            angle = interpolation_list[0].start_theta_list[i]
            dubin_paths = dubins_path_planner([path[0][0],path[0][1],start_theta],[path[1][0],path[1][1],angle],self.limo_turning_rad)
            end_theta_list.append(angle)

            # Get dubin path with minimum length
            min_dubin_length = 1000000
            for dubin_path in dubin_paths:
                length = 0
                have_straight_segment = False
                for dubin_segment in dubin_path:
                    if dubin_segment.dir == 0:
                        have_straight_segment = True
                    length += dubin_segment.length
                if have_straight_segment:
                    mul = 2
                else:
                    mul = 4    
                if min_dis_list[index] < mul*self.limo_turning_rad:                  
                    sample_pose = sample_path(dubin_path, [path[0][0],path[0][1],theta],self.limo_turning_rad/8,self.limo_turning_rad)
                    collision = False
                    for pose in sample_pose:

                        for obstacle in self.offset_obstacles:
                            if point_in_polygon([pose[0],pose[1]],obstacle):
                                collision = True
                                break

                        if not point_in_polygon([pose[0],pose[1]],self.offset_map):
                            collision = True
                        
                        if collision:
                            length += 10000
                            break
                
                if length < min_dubin_length:
                    min_dubin_length = length
                    min_path = dubin_path
                    
                    # End get dubin path with minimum length
                    
            min_value = min_dubin_length + interpolation_list[0].min_value_list[i]
            value_list.append(min_value)
            path_list.append(min_path)
            
        min_value = 100000
        for i in range(len(value_list)):
            if min_value > value_list[i]:
                min_value = value_list[i]
                min_index = i
        min_path = path_list[min_index]
        end_angle = end_theta_list[min_index]
        n = dubin_interpolation([min_value],[start_theta],[end_angle],[min_path])
        interpolation_list.insert(0,n)
        return interpolation_list
    
    def get_path_from_interpolation(self,interpolation_list):
        
        path = []
        current_theta = interpolation_list[0].start_theta_list[0]
        cost = interpolation_list[0].min_value_list[0]
        for i in range(len(interpolation_list)-1):
            index = interpolation_list[i].start_theta_list.index(current_theta)
            sub_path = interpolation_list[i].path_list[index]
            for seg in sub_path:
                path.append(seg)
            current_theta = interpolation_list[i].end_theta_list[index]
        return path, cost
    
    
    def find_path(self, limo0,limo1, goal):
        
        self.polygon_offsetting()
        self.create_voronoi_roadmap()
        # Connecting start and goal points to the roadmap

        start_dis_list = []
        end_dis_list = []
        for i in range(len(self.voronoi_vertices)):
            x = self.voronoi_vertices[i].x
            y = self.voronoi_vertices[i].y
            start_dis = ((limo0[0]-x)**2 + (limo0[1]-y)**2)*0.5
            goal_dis = ((goal[0]-x)**2 + (goal[1]-y)**2)*0.5
            edge1 = Edge(np.array( [np.array([limo0[0],limo0[1]]) , np.array([x,y]) ] ) )
            edge2 = Edge(np.array( [np.array([goal[0],goal[1]]) , np.array([x,y]) ] ) )
            check1 = False
            check2 = False
            for obstacle in self.offset_obstacles:
                for ob_edge in obstacle.edge:
                    if test_edge_intersect(ob_edge,edge1):
                        start_dis += 10000
                        check1 = True
                        break
                if check1:
                    break

            for obstacle in self.offset_obstacles:
                for ob_edge in obstacle.edge:
                    if test_edge_intersect(ob_edge,edge2):
                        goal_dis += 10000
                        check2 = True
                        break
                if check2:
                    break
            start_dis_list.append(start_dis)
            end_dis_list.append(goal_dis)
            # if min_dis1 > start_dis:
            #     min_dis1 = start_dis
            #     node_index1 = i
            # if min_dis2 > goal_dis:
            #     min_dis2 = goal_dis
            #     node_index2 = i
        self.limo0_path_found = False
        while start_dis_list and not self.limo0_path_found:
            # res = np.where(start_dis_list == np.min(start_dis_list))[0]
            # start_cost = start_dis_list.pop(res[0]) 
            node_index1 = start_dis_list.index(min(start_dis_list))
            start_dis_list.pop(node_index1)
            while end_dis_list and not self.limo0_path_found:
                # res = np.where(end_dis_list == np.min(end_dis_list))[0]
                # end_cost = end_dis_list.pop(res[0])
                node_index2 = end_dis_list.index(min(end_dis_list))
                end_dis_list.pop(node_index2)

                voronoi_edges = copy.deepcopy(self.voronoi_edges)
                voronoi_vertices = copy.deepcopy(self.voronoi_vertices)

                # Start
                x = voronoi_vertices[node_index1].x
                y = voronoi_vertices[node_index1].y
                edge = Edge(np.array( [ np.array([limo0[0],limo0[1]]) , np.array([x,y]) ] ) )
                start_node = Node(limo0[0],limo0[1])
                dis = self.get_min_dis_to_edge(edge)
                start_node.addEdge(node_index1)
                start_node.addMinDis(dis)
                voronoi_vertices.append(start_node)
                voronoi_vertices[node_index1].addEdge(len(voronoi_vertices)-1)
                voronoi_vertices[node_index1].addMinDis(dis)
                voronoi_edges.append(edge)
                # Goal
                
                x = voronoi_vertices[node_index2].x
                y = voronoi_vertices[node_index2].y
                edge = Edge(np.array( [ np.array([goal[0],goal[1]]) , np.array([x,y]) ] ) )
                goal_node = Node(goal[0],goal[1])
                dis = self.get_min_dis_to_edge(edge)
                goal_node.addEdge(node_index2)
                goal_node.addMinDis(dis)
                voronoi_vertices.append(goal_node)
                voronoi_vertices[node_index2].addEdge(len(voronoi_vertices)-1)
                voronoi_vertices[node_index2].addMinDis(dis)
                voronoi_edges.append(edge)

                # Call Dijkstra to find path
                path_index = dijkstra(voronoi_vertices,len(voronoi_vertices)-2,len(voronoi_vertices)-1)
                interpolation_list = self.interpolation(voronoi_vertices,path_index,limo0[2],self.gates[2],np.pi/9)
                self.limo0_path, cost = self.get_path_from_interpolation(interpolation_list)
                if cost < 1000:
                    self.limo0_path_found = True
                    c = limo0
                    for seg in self.limo0_path:
                        c = get_current_pose(seg,c,self.limo_turning_rad)
                    print(c)
                    print("Fininshed get path for limo0, getting path for limo1")
        if not self.limo0_path_found:
            print("No path for limo0")
            self.limo0_path = [Dubin_path_segment(0,0)]
        
        # Finished find path for limo0, finding path for limo1
        resolution = 0.8
        self.approximate_cell_decomposition(resolution)

        theta = np.pi/2 -self.gates[2]
        [x,y] = rotate(theta,[self.gates[0],self.gates[1]])
        rotated_gate = [x,y,np.pi/2]

        [x,y] = rotate(theta,[limo0[0],limo0[1]])
        rotated_limo0 = [x,y,(limo0[2] + theta)%(2*np.pi)]

        [x,y] = rotate(theta,[limo1[0],limo1[1]])
        rotated_limo1 = [x,y,(limo1[2] + theta)% (2*np.pi)]

        # Get k latice closest to limo1
        temp = [10000,10000,0]
        closest_limo1 = []
        for i in range(5):
            closest_limo1.append(temp)

        for i in range(self.horizontal_cell_number):
            for k in range(self.vertical_cell_number):
                for n in range(0,2):
                    for m in range(0,2):
                        if n == 1 and m == 0:
                            x,y = self.x_min + resolution*i + n*resolution/2, self.y_min + resolution*k + m*resolution/2
                            closest_limo1.append([x,y,np.pi/2])
                            closest_limo1.append([x,y,3*np.pi/2])
                            closest_limo1.sort(key= lambda x: ((x[0] - rotated_limo1[0])**2 + (x[1] - rotated_limo1[1])**2)**0.5 )
                            closest_limo1.pop(len(closest_limo1)-1)
                            closest_limo1.pop(len(closest_limo1)-1)
                        elif n == 0 and m ==1:
                            x,y = self.x_min + resolution*i + n*resolution/2, self.y_min + resolution*k + m*resolution/2
                            closest_limo1.append([x,y,0])
                            closest_limo1.append([x,y,np.pi])
                            closest_limo1.sort(key= lambda x: ((x[0] - rotated_limo1[0])**2 + (x[1] - rotated_limo1[1])**2)**0.5 )
                            closest_limo1.pop(len(closest_limo1)-1)
                            closest_limo1.pop(len(closest_limo1)-1)
        
        # End get k closest latice to limo1
        path_list = []
        length_list = []
        for latice in closest_limo1:
            dubin_paths = dubins_path_planner(rotated_limo1,latice,self.limo_turning_rad)
            for dubin_path in dubin_paths:
                sample_pose = sample_path(dubin_path, rotated_limo1,self.limo_turning_rad/5,self.limo_turning_rad)
                collision = False
                for pose in sample_pose:
                    for obstacle in self.rotated_obstacles:
                        if point_in_polygon([pose[0],pose[1]],obstacle):
                            collision = True
                            break
                    if collision:
                        break
                    else:
                        if not point_in_polygon([pose[0],pose[1]],self.rotated_map):
                            collision = True
                            break
                if collision:
                    continue

                # Get limo0 path during dubin_path period
                length = dubin_path_length(dubin_path)
                i = 0
                limo0_path = []
                if i < len(self.limo0_path):
                    l = length
                    while  l >= self.limo0_path[i].length and i < len(self.limo0_path):
                        l -= self.limo0_path[i].length
                        limo0_path.append(self.limo0_path[i])
                        i += 1
                        if i >= len(self.limo0_path):
                            break
                    if i < len(self.limo0_path):
                        limo0_path.append(self.limo0_path[i])   
                # End get limo0 path during dubin_path period

                if not self.check_collision_2_limo_path(rotated_limo0,limo0_path,rotated_limo1,dubin_path):
                    path_list.append(dubin_path)
                    length_list.append(length)
        path_list = sorted(path_list, key=lambda x : length_list[path_list.index(x)])
        length_list.sort()
        self.found_limo1_path = False
        while not self.found_limo1_path and path_list:
            limo1_current_pose = rotated_limo1
            limo1_path = path_list.pop(0)
            length = length_list.pop(0)
            self.limo1_path = []
            
            for seg in limo1_path:
                limo1_current_pose = get_current_pose(seg,limo1_current_pose,self.limo_turning_rad)
                self.limo1_path.append(seg)

            # Get limo0 pose and remaining path when limo1 reached latice

            limo0_current_pose,seg_index,left_over_seg_length = self.get_pose_at_time(rotated_limo0,self.limo0_path,length)
            limo0_remaining_path = []
            limo0_remaining_path.append(Dubin_path_segment(self.limo0_path[seg_index].dir,left_over_seg_length))
            for i in range(seg_index+1,len(self.limo0_path)):
                limo0_remaining_path.append(self.limo0_path[i])
            # Finish get limo0 pose and remaining path when limo1 reached latice

            # Test to check if path for limo1 exist first
            p , check= self.limo1_path_exist(limo1_current_pose,rotated_gate ,self.x_min,self.y_min,self.cell_array,resolution,self.vertical_cell_number)
            if check:
                print("Path exist, Checking path")
                path = []
                path_reference = []
                current_node = [limo1_current_pose[0],limo1_current_pose[1],limo1_current_pose[2]]
                for dpath in p:
                    n = Simple_Node2(current_node[0],current_node[1],current_node[2])
                    path_reference.append(n)
                    for seg in dpath:
                        current_node = get_current_pose(seg,current_node,self.limo_turning_rad)
                        path.append(seg)    
                    
                if self.check_collision_2_limo_path(limo0_current_pose, limo0_remaining_path,limo1_current_pose,path):
                    print("Path collide with limo0, finding new path")
                    
                    # Call A_star to search path for limo1 on latice
                    self.found_limo1_path , a = self.A_star(limo0_current_pose,limo0_remaining_path, limo1_current_pose,rotated_gate ,self.x_min,self.y_min,self.cell_array,resolution,self.vertical_cell_number, path_reference)
                    if self.found_limo1_path:
                        for dpath in a:
                            for seg in dpath:
                                self.limo1_path.append(seg)
                    else:
                        print("Path not found, changing to different start latice")
                else:
                    self.found_limo1_path = True
                    for seg in path:
                        self.limo1_path.append(seg)
            else:
                print("No path for limo1, changing to different start latice")
        
        if self.found_limo1_path:
            sample2 = sample_path(self.limo1_path,limo1,self.limo_turning_rad/4, self.limo_turning_rad)
            self.plot_path(sample2)
        else:
            self.limo1_path = [Dubin_path_segment(0,0)]
        sample = sample_path(self.limo0_path, limo0, self.limo_turning_rad/4, self.limo_turning_rad)
        self.plot_offset_map()
        self.plot_path(sample)                

        
    def get_min_dis_to_edge(self,edge):
        
        min_dis = 10000
        sample_edge = sample_point_on_edge(edge,11)
        for map_edge in self.offset_map.edge:
            for i in range(len(sample_edge)):
                dis = get_distance_point_2_edge(sample_edge[i],map_edge)
                if min_dis > dis:
                    min_dis = dis

        for obstacle in self.offset_obstacles:
            for ob_edge in obstacle.edge:
                for i in range(len(sample_edge)):
                    dis = get_distance_point_2_edge(sample_edge[i],ob_edge)
                    if dis < min_dis:
                        min_dis = dis
        return min_dis
    
    def get_limo_vertices(self, current_pose):
        
        location = np.array([current_pose[0],current_pose[1]])

        rotational_matrix = np.array(
            [np.array([math.cos(current_pose[2]),-math.sin(current_pose[2])]),
            np.array([math.sin(current_pose[2]),math.cos(current_pose[2])])]
        )
        
        current_vertices = np.transpose(rotational_matrix@self.limo_vertices)
        for i in range(len(current_vertices)):
            current_vertices[i] += location
        return current_vertices
    
    def check_collision_2_limo_pose(self, limo0_pose, limo1_pose):
            
        # Broad phase

        narrow_phase = False
        
        dis = ((limo0_pose[0] - limo1_pose[0])**2 + (limo0_pose[1] - limo1_pose[1])**2)**0.5

        if dis > 2*self.limo_radius:
            return False
        else:
            narrow_phase = True

        # Narrow phase

        if narrow_phase:

            limo0_vertices = self.get_limo_vertices(limo0_pose)
            limo0 = Obstacle(limo0_vertices)

            limo1_vertices = self.get_limo_vertices(limo1_pose)
            limo1 = Obstacle(limo1_vertices)
            for edge0 in limo0.edge:
                for edge1 in limo1.edge:
                    check = test_edge_intersect(edge0, edge1)
                    if check:
                        return True
        return False
    
    def check_collision_2_limo_path(self, limo0_ini_pose, limo0_path, limo1_ini_pose, limo1_path):
        
        limo0_sample_pose = sample_path(limo0_path,limo0_ini_pose,self.limo_turning_rad/5,self.limo_turning_rad)
        limo1_sample_pose = sample_path(limo1_path,limo1_ini_pose,self.limo_turning_rad/5,self.limo_turning_rad)
        
        if len(limo0_sample_pose) > len(limo1_sample_pose):
            for i in range(len(limo1_sample_pose)):
                check = self.check_collision_2_limo_pose(limo0_sample_pose[i],limo1_sample_pose[i])
                if check:
                    return True
        else:
            for i in range(len(limo0_sample_pose)):
                check = self.check_collision_2_limo_pose(limo0_sample_pose[i],limo1_sample_pose[i])
                if check:
                    return True
        return False


    
    def polygon_offsetting(self):
        
        # Offsetting Obstacle

        for obstacle in self.obstacles:
            offset_obstacle_vertices = []
            for i in range(len(obstacle.edge)):
                if i == 0:
                    vertice = obstacle.edge[i].vertices[0]
                    edge1 = obstacle.edge[len(obstacle.edge)-1].edge
                    edge2 = -obstacle.edge[i].edge
                    normal1 = obstacle.edge[len(obstacle.edge)-1].normal
                    normal2 = obstacle.edge[i].normal
                else:
                    vertice = obstacle.edge[i].vertices[0]
                    edge1 = obstacle.edge[i-1].edge
                    edge2 = -obstacle.edge[i].edge
                    normal1 = obstacle.edge[i-1].normal
                    normal2 = obstacle.edge[i].normal
                    
                # Get outward normal
                if normal1.T @ edge2 >= 0 :
                    normal1 = -normal1
                if normal2.T @ edge1 >= 0 :
                    normal2 = -normal2
                
                # Get offset vertice
                vector = normal1 + normal2
                vector = vector/(vector[0]**2+vector[1]**2)**0.5
                l = self.limo_radius/((1 + normal1[0]*normal2[0]+normal1[1]*normal2[1])/2)**0.5
                offset_vertice = vertice + l*vector
                offset_obstacle_vertices.append(offset_vertice)
            offset_obstacle = Obstacle(offset_obstacle_vertices)
            self.offset_obstacles.append(offset_obstacle)

        # Offsetting map

        offset_map_vertices = []
        for i in range(len(self.map.edge)):
            if i == 0:
                vertice = self.map.edge[i].vertices[0]
                edge1 = self.map.edge[len(self.map.edge)-1].edge
                edge2 = -self.map.edge[i].edge
                normal1 = self.map.edge[len(self.map.edge)-1].normal
                normal2 = self.map.edge[i].normal
            else:
                vertice = self.map.edge[i].vertices[0]
                edge1 = self.map.edge[i-1].edge
                edge2 = -self.map.edge[i].edge
                normal1 = self.map.edge[i-1].normal
                normal2 = self.map.edge[i].normal
                
            # Get inward normal
            if normal1.T @ edge2 <= 0 :
                normal1 = -normal1
            if normal2.T @ edge1 <= 0 :
                normal2 = -normal2
            
            # Get offset vertice
            vector = normal1 + normal2
            vector = vector/(vector[0]**2+vector[1]**2)**0.5
            l = self.limo_radius/((1 + normal1[0]*normal2[0]+normal1[1]*normal2[1])/2)**0.5
            offset_vertice = vertice + l*vector
            offset_map_vertices.append(offset_vertice)
        
        self.offset_map = Obstacle(offset_map_vertices)
        

if __name__ == '__main__':
    
    map_border_vertices = [
        np.array([-5,5]),
        np.array([5,5]),
        np.array([5,-5]),
        np.array([-5,-5])
    ]
    
    ob_list = []

    ob_vertices = [
        np.array([0,0.5]),
        np.array([2,0.5]),
        np.array([2,3.5]),
        np.array([0,3.5])
    ]
    ob = Obstacle(ob_vertices)

    ob2_vertices = [
        np.array([-3,-4]),
        np.array([-3,-3]),
        np.array([-1,-3]),
        np.array([-1,-4])
    ]
    ob2 = Obstacle(ob2_vertices)
    ob_list.append(ob)
    ob_list.append(ob2)
    gates = [3,3,3*np.pi/2]
    limo0 = [-4,0,0]
    limo1 = [-1.5,-1,0]
    
    map = Map(map_border_vertices,ob_list,gates)

    print("-----------------------")

    map.find_path(limo0,limo1,gates)
    plt.show()

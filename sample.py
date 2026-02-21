import numpy as np
import math
import matplotlib.pyplot as plt
from dubin import dubins_path_planner,get_current_pose, sample_path
from utils2 import test_edge_intersect, node_in_list, get_distance_point_2_edge, get_angle


class Dubin_path_segment():

    def __init__(self,direction,length):

        self.dir = direction
        self.length = length

class Edge():
    
    def __init__(self, vertices):
        
        self.vertices = vertices
        self.edge  = vertices[0] - vertices[1]
        if self.edge[1] != 0:
            a = 1/(1+pow(self.edge[0]/self.edge[1],2))
            a = pow(a,0.5)
            b = -a*self.edge[0]/self.edge[1]
        else:
            b = 1/(1+pow(self.edge[1]/self.edge[0],2))
            b = pow(b,0.5)
            a = -b*self.edge[1]/self.edge[0]
        self.normal = np.array([a,b])


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
                if i != k:
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


class Node:

    def __init__(self,x,y,ori,child,path_segment_to_child):
        self.x = x
        self.y = y
        self.theta = (ori) % (2 *np.pi)
        self.child = child
        self.path = path_segment_to_child

    def __eq__(self, ob):
        return ((round(self.x - ob.x,3) == 0) and (round(self.y - ob.y,3) == 0)  and (round(self.theta - ob.theta,3) == 0)  and (self.child == ob.child) and (self.path == ob.path))


class Map():

    def __init__(self,map_border_vertices, obstacles_list, gates,limo0,limo1):
        self.limo_turning_rad = 0.2
        self.map = Obstacle(map_border_vertices)
        self.obstacles = obstacles_list
        self.limo_vertices = np.array(
            [np.array([0.3,0.15]),
            np.array([0.3,-0.15]),
            np.array([-0.3,-0.15]),
            np.array([-0.3,0.15])]
        )
        self.limo_vertices = np.transpose(self.limo_vertices)
        self.limo_radius = 0.335
        self.limo_turning_rad = 0.4
        self.gates = gates
        self.node_list = []

        self.limo0 = limo0
        self.limo1 = limo1

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

    def plot_path(self,sample_path):
        for i in range(len(sample_path)-1):
            x = [sample_path[i][0],sample_path[i+1][0]]
            y = [sample_path[i][1],sample_path[i+1][1]]
            plt.plot(x,y)

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


 
    def collision_detection(self,dubin_path_segment,initial_pose):
        # Sampling the path segment by a predetermined interval
        
        prev_pose = initial_pose
        number_sample = 10
        for i in range(0,number_sample):
            
            test_interval = Dubin_path_segment(direction=dubin_path_segment.dir, length = dubin_path_segment.length/number_sample)
            # Calulating the pose of the current interval
            current_pose = get_current_pose(test_interval,prev_pose, self.limo_turning_rad)

            # Check collision with obstacles
            
            # Broad phase
            narrow_phase_ob = False
            for obstacle in self.obstacles:
                min_dis = 10000
                for edge in obstacle.edge:
                    dis = get_distance_point_2_edge(current_pose,edge)
                    if dis < min_dis:
                        min_dis = dis
                if min_dis < self.limo_radius:
                    narrow_phase_ob = True
            # Narrow phase
            if narrow_phase_ob:
                limo_vertices = self.get_limo_vertices(current_pose)
                for obstacle in self.obstacles:
                    for k in range(len(limo_vertices)):
                        if k < len(limo_vertices)-1:
                            limo_edge = Edge([limo_vertices[k],limo_vertices[k+1]])
                        else:
                            limo_edge = Edge([limo_vertices[k],limo_vertices[0]])

                        # Check each edge of limo collision with each edge of the obstales
                        for edge in obstacle.edge:
                            check = test_edge_intersect(limo_edge, edge)
                            if check:
                                return True
            # ChecK collision with map borders
             
            # Broad phase
            narrow_phase_border = False
            min_dis = 10000
            for edge in self.map.edge: 
                dis = get_distance_point_2_edge(current_pose,edge)
                if dis < min_dis:
                    min_dis = dis
            if min_dis < self.limo_radius:
                narrow_phase_border = True
            # Narrow phase
            if narrow_phase_border:
                limo_vertices = self.get_limo_vertices(current_pose)
                for k in range(len(limo_vertices)):
                    if k < len(limo_vertices)-1:
                        limo_edge = Edge([limo_vertices[k],limo_vertices[k+1]])
                    else:
                        limo_edge = Edge([limo_vertices[k],limo_vertices[0]])

                    # Check each edge of limo collision with each edge of the obstales
                    for edge in self.map.edge:
                        check = test_edge_intersect(limo_edge, edge)
                        if check:
                            return True
            prev_pose = current_pose
        return False

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

    def get_roadmap(self):

        # Divide map into 50x50 rectangle cells

        x_min = 10000
        x_max = -10000
        y_min = 10000
        y_max = -10000
        for vertice in self.map.vertices:
            
            if x_min > vertice[0]:
                x_min = vertice[0]
                
            if x_max < vertice[0]:
                x_max = vertice[0]

            if y_min > vertice[1]:
                y_min = vertice[1]

            if y_max < vertice[1]:
                y_max = vertice[1]
        
        
        cell_width = (x_max-x_min)/50
        cell_height = (y_max-y_min)/50

        check_cell_array = np.zeros((50,50))
        
        gates_node = Node(self.gates[0],
                            self.gates[1],
                            self.gates[2],
                            [],[])
        self.node_list.append(gates_node)

        closed_list = []
        for i in range(9):  
            new_node = []
            for child_node in self.node_list:
                if not node_in_list(child_node,closed_list):
            
                    x = child_node.x
                    y = child_node.y
                    theta = child_node.theta

                    # Inverse current orientation
                    if theta <= math.pi:
                        theta2 = theta + math.pi
                    else:
                        theta2 = theta - math.pi
                    
                    # Generate straight node 
                    
                    seg = Dubin_path_segment(0,0.5*math.pi*self.limo_turning_rad)
                    [new_x, new_y, new_theta] = get_current_pose(seg,[x,y,theta2], self.limo_turning_rad)


                    cell_width_index = int((new_x - x_min)/cell_width)
                    cell_height_index = int((new_y- y_min)/cell_height)

                    if cell_width_index < 50 and cell_height_index <50:
                        if ((not self.collision_detection(seg,[new_x,new_y,theta])) 
                        and (check_cell_array[cell_height_index][cell_width_index] == 0)):
                            new_node.append(Node(new_x,new_y,theta, child_node,seg))
                            
                            check_cell_array[cell_height_index][cell_width_index] += 1

                    # Generate Left node (inverse Right)
                    seg = Dubin_path_segment(-1,0.5*math.pi*self.limo_turning_rad)
                    [new_x, new_y, new_theta] = get_current_pose(seg,[x,y,theta2],self.limo_turning_rad)
                    inverse_seg = Dubin_path_segment(1,0.5*math.pi*self.limo_turning_rad)
                    if new_theta <= math.pi:
                        new_theta +=  math.pi
                    else:
                        new_theta -= math.pi
                    
                    cell_width_index = int((new_x - x_min)/cell_width)
                    cell_height_index = int((new_y- y_min)/cell_height)
                    if cell_width_index < 50 and cell_height_index <50:
                        if ((not self.collision_detection(inverse_seg,[new_x,new_y,new_theta]))
                            and (check_cell_array[cell_height_index][cell_width_index] == 0)):
                            
                            new_node.append(Node(new_x,new_y,new_theta,child_node,inverse_seg))

                            check_cell_array[cell_height_index][cell_width_index] += 1


                    # Generate Right node (inverse Left)
                    seg = Dubin_path_segment(1,0.5*math.pi*self.limo_turning_rad)
                    [new_x, new_y, new_theta] = get_current_pose(seg,[x,y,theta2],self.limo_turning_rad)
                    inverse_seg = Dubin_path_segment(-1,0.5*math.pi*self.limo_turning_rad)
                    if new_theta <= math.pi:
                        new_theta +=  math.pi
                    else:
                        new_theta -= math.pi

                    cell_width_index = int((new_x - x_min)/cell_width)
                    cell_height_index = int((new_y- y_min)/cell_height)
                    if cell_width_index < 50 and cell_height_index <50:
                        if ((not self.collision_detection(inverse_seg,[new_x,new_y,new_theta]))
                            and (check_cell_array[cell_height_index][cell_width_index] == 0)):

                            new_node.append(Node(new_x,new_y,new_theta,child_node,inverse_seg))

                            check_cell_array[cell_height_index][cell_width_index] += 1


                    closed_list.append(child_node)

            for node in new_node:
                self.node_list.append(node)
        return self.node_list

    
    def distance_limo0(self,node):
        return ((node.x-self.limo0[0])**2 + (node.y-self.limo0[1])**2)**0.5
    
    def distance_limo1(self,node):
        return ((node.x-self.limo1[0])**2 + (node.y-self.limo1[1])**2)**0.5
    

    def find_path(self):

        self.get_roadmap()
        if not self.node_list:
            return None, None, None
        self.roadmap = self.node_list
        
        # Get 5 closest node to limo 0 and limo 1
        
        temp_node = Node(1000,1000,0,[],[])

        closest_limo0 = []
        closest_limo1 = []
        for i in range(5):
            closest_limo0.append(temp_node)
            closest_limo1.append(temp_node)

        for node in self.roadmap:
            closest_limo0.append(node)
            closest_limo0.sort(key = self.distance_limo0)
            closest_limo0.pop(len(closest_limo0)-1)

            closest_limo1.append(node)
            closest_limo1.sort(key = self.distance_limo1)
            closest_limo1.pop(len(closest_limo1)-1)

        # Test collision with each path pair
        
        limo0_path_list = []
        limo1_path_list = []
        cost_list = []
        for node0 in closest_limo0:
            # Get path for limo0
            paths0 = dubins_path_planner(self.limo0, [node0.x,node0.y,node0.theta],self.limo_turning_rad)

            for path0 in paths0:
                current_pose = self.limo0
                limo0_path = []
                limo0_path_length = 0
                check0 = False
                for seg in path0:

                    # Test collision for BVP

                    if self.collision_detection(seg,current_pose):
                        check0 = True
                        break
                    else:
                        current_pose = get_current_pose(seg,current_pose,self.limo_turning_rad)
                        limo0_path_length += seg.length
                        limo0_path.append(seg)
                if check0:
                    continue
                else:
                    
                    # Add the remaining path from roadmap
                    next_node = node0
                    
                    while next_node != self.roadmap[0]:
                        limo0_path_length += next_node.path.length
                        limo0_path.append(next_node.path)
                        next_node = next_node.child
                    
                    for node1 in closest_limo1:

                        # Get path for limo 1 
                        
                        paths1 = dubins_path_planner(self.limo1, [node1.x,node1.y,node1.theta],self.limo_turning_rad)

                        
                        for path1 in paths1:
                            current_pose = self.limo1
                            limo1_path = []
                            check1 = False
                            limo1_path_length = 0
                            for seg in path1:

                                # Test collision for BVP

                                if self.collision_detection(seg,current_pose):
                                    check1 = True
                                    break
                                else:
                                    current_pose = get_current_pose(seg,current_pose,self.limo_turning_rad)
                                    limo1_path_length += seg.length
                                    limo1_path.append(seg)
                            if check1:
                                continue
                            else:
                                # Add the path from roadmap
                                next_node = node1
                                while next_node != self.roadmap[0]:
                                    limo1_path_length += next_node.path.length
                                    limo1_path.append(next_node.path)
                                    next_node = next_node.child
                                    
                            collision = self.check_collision_2_limo_path(self.limo0,limo0_path,self.limo1,limo1_path)
                            
                            if not collision:
                                limo0_path_list.append(limo0_path)
                                limo1_path_list.append(limo1_path)
                                cost_list.append(max(limo0_path_length,limo1_path_length))
        if cost_list:
            min_index = cost_list.index(min(cost_list))
            return limo0_path_list[min_index],limo1_path_list[min_index], cost_list[min_index]
        else:
            return None,None,10000

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
    gates = [3,3,4.73]
    limo0 = [-4,0,0]
    limo1 = [-1.5,-1,0]
    
    map = Map(map_border_vertices,ob_list,gates, limo0,limo1)
    limo0_path, limo1_path, cost = map.find_path()
    sample1 = sample_path(limo0_path,limo0,map.limo_turning_rad/4,map.limo_turning_rad)
    sample2 = sample_path(limo1_path,limo1,map.limo_turning_rad/4,map.limo_turning_rad)
    map.plot_map()
    map.plot_path(sample1)
    map.plot_path(sample2)

    plt.show()

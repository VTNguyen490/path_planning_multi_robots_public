import numpy as np
import math

import pyvoronoi
import matplotlib.pyplot as plt
import copy

def get_angle(vector1,vector2):
    angle = math.atan2(vector2[1], vector2[0]) - math.atan2(vector1[1], vector1[0])
    if angle < 0:
        angle += 2*math.pi
    return angle

def node_in_list(node,list):
    
    if list:
        return False
    else:
        if node in list:
            return True
        return False

def test_edge_intersect(edge1, edge2):

    if (edge1.vertices[1][0] - edge1.vertices[0][0])*(edge2.vertices[1][0] - edge2.vertices[0][0]) != 0:
        a1 = (edge1.vertices[1][1] - edge1.vertices[0][1])/(edge1.vertices[1][0] - edge1.vertices[0][0])
        b1 = edge1.vertices[1][1]-a1*edge1.vertices[1][0]

        a2 = (edge2.vertices[1][1] - edge2.vertices[0][1])/(edge2.vertices[1][0] - edge2.vertices[0][0])
        b2 = edge2.vertices[1][1]-a2*edge2.vertices[1][0]

        if a1 == a2 and b1 == b2:
            if ((edge1.vertices[0][0]-edge2.vertices[0][0])*(edge1.vertices[0][0]-edge2.vertices[1][0]) <= 0
                or (edge1.vertices[1][0]-edge2.vertices[0][0])*(edge1.vertices[0][0]-edge2.vertices[1][0]) <= 0):
                return True
            else:
                return False
            
        elif a1 == a2 and b1 != b2:
            return False
        else:
            x = -(b2-b1)/(a2-a1)
            if ((x-edge1.vertices[0][0])*(x-edge1.vertices[1][0]) <= 0
            and (x-edge2.vertices[0][0])*(x-edge2.vertices[1][0]) <= 0):
                return True
            else:
                return False
            
    elif (edge1.vertices[1][0] - edge1.vertices[0][0]) == 0 and (edge2.vertices[1][0] - edge2.vertices[0][0]) != 0:
        a2 = (edge2.vertices[1][1] - edge2.vertices[0][1])/(edge2.vertices[1][0] - edge2.vertices[0][0])
        b2 = edge2.vertices[1][1]-a2*edge2.vertices[1][0]

        x = edge1.vertices[0][0]
        y = a2*x+b2

        if ((x-edge2.vertices[0][0])*(x-edge2.vertices[1][0]) <= 0
            and (y-edge1.vertices[0][1])*(y-edge1.vertices[1][1]) <= 0):
            return True
        else:
            return False

    elif (edge1.vertices[1][0] - edge1.vertices[0][0]) != 0 and (edge2.vertices[1][0] - edge2.vertices[0][0]) == 0:
        a1 = (edge1.vertices[1][1] - edge1.vertices[0][1])/(edge1.vertices[1][0] - edge1.vertices[0][0])
        b1 = edge1.vertices[1][1]-a1*edge1.vertices[1][0]

        x = edge2.vertices[0][0]
        y = a1*x+b1

        if ((x-edge1.vertices[0][0])*(x-edge1.vertices[1][0]) <= 0
            and (y-edge2.vertices[0][1])*(y-edge2.vertices[1][1]) <= 0):
            return True
        else:
            return False
    
    else:
        if edge1.vertices[0][0] != edge2.vertices[0][0]:
            return False
        else:
            if ((edge1.vertices[0][1]-edge2.vertices[0][1])*(edge1.vertices[0][1]-edge2.vertices[1][1]) <= 0
                or (edge1.vertices[1][1]-edge2.vertices[0][1])*(edge1.vertices[1][1]-edge2.vertices[1][1]) <= 0):
                return True
            else:
                return False
            
def get_current_pose(dubin_path_segment, initial_pose, turning_rad):
        
    if dubin_path_segment.dir == 1:     
        direct = [math.cos(initial_pose[2]),math.sin(initial_pose[2])]
        c = cross_product([-math.sin(initial_pose[2]),math.cos(initial_pose[2])],direct)
        if c < 0 :
            x = initial_pose[0]+(math.sin(initial_pose[2])-math.sin(initial_pose[2]-dubin_path_segment.length/turning_rad))*turning_rad
            y = initial_pose[1]-(math.cos(initial_pose[2])-math.cos(initial_pose[2]-dubin_path_segment.length/turning_rad))*turning_rad
            phi = initial_pose[2]-dubin_path_segment.length/turning_rad

    
        elif c >= 0:
            x = initial_pose[0]+(math.sin(initial_pose[2])-math.sin(initial_pose[2]+dubin_path_segment.length/turning_rad))*turning_rad
            y = initial_pose[1]-(math.cos(initial_pose[2])-math.cos(initial_pose[2]+dubin_path_segment.length/turning_rad))*turning_rad
            phi = initial_pose[2]+dubin_path_segment.length/turning_rad

    elif dubin_path_segment.dir == -1:
        direct = [math.cos(initial_pose[2]),math.sin(initial_pose[2])]
        c = cross_product([math.sin(initial_pose[2]),-math.cos(initial_pose[2])],direct)
        if c < 0 :
            x = initial_pose[0] - (math.sin(initial_pose[2])-math.sin(initial_pose[2]-dubin_path_segment.length/turning_rad))*turning_rad
            y = initial_pose[1] + (math.cos(initial_pose[2])-math.cos(initial_pose[2]-dubin_path_segment.length/turning_rad))*turning_rad
            phi = initial_pose[2]-dubin_path_segment.length/turning_rad
        elif c >= 0:
            x = initial_pose[0]-(math.sin(initial_pose[2])-math.sin(initial_pose[2]+dubin_path_segment.length/turning_rad))*turning_rad
            y = initial_pose[1]+(math.cos(initial_pose[2])-math.cos(initial_pose[2]+dubin_path_segment.length/turning_rad))*turning_rad
            phi = initial_pose[2]+dubin_path_segment.length/turning_rad
    
    elif dubin_path_segment.dir == 0:
        x = initial_pose[0] + math.cos(initial_pose[2])*dubin_path_segment.length
        y = initial_pose[1] + math.sin(initial_pose[2])*dubin_path_segment.length
        phi = initial_pose[2]

    if phi < 0:
        phi += 2*math.pi
    if phi > 2*math.pi:
        phi -= 2*math.pi

    return [x,y,phi]


def cross_product(x,y):
    return x[0]*y[1]-x[1]*y[0]

def get_distance_point_2_edge(point, edge):
    position = np.array([point[0],point[1]])
    dis = np.dot(edge.vertices[0]-position,edge.normal) 
    
    # Test if projection is inside edge
    proj_position = position + dis*edge.normal
    if edge.vertices[0][0] == edge.vertices[1][0]:
        if (proj_position[1]-edge.vertices[0][1])*(proj_position[1]-edge.vertices[1][1]) <= 0:
            point_on_edge = True
        else:
            point_on_edge = False
    elif edge.vertices[0][1] == edge.vertices[1][1]:
        if (proj_position[0]-edge.vertices[0][0])*(proj_position[0]-edge.vertices[1][0]) <= 0:
            point_on_edge = True
        else:
            point_on_edge = False
    else:
        x = edge.vertices[1][1] - ((edge.vertices[1][0] - edge.vertices[0][0]))*(edge.vertices[1][1] - proj_position[1])/(edge.vertices[1][1] - edge.vertices[0][1])
        if round(x,4) == round(proj_position[0],4):
            point_on_edge = True
        else:
            point_on_edge = False

    if point_on_edge:
        return abs(dis)
    else:
        vect1 = position - edge.vertices[0]
        dis1 = (vect1[0]**2+vect1[1]**2)**0.5

        vect2 = position - edge.vertices[1]
        dis2 = (vect2[0]**2+vect2[1]**2)**0.5

        return min([dis1,dis2])
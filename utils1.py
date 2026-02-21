import numpy as np
import math
import copy


def get_angle(vector1,vector2):
    angle = math.atan2(vector2[1], vector2[0]) - math.atan2(vector1[1], vector1[0])
    if angle < 0:
        angle += 2*math.pi
    return angle

def get_abs_angle(vector):
    angle = math.atan2(vector[1], vector[0])
    if angle < 0:
        angle += 2*math.pi
    return angle

def sample_point_on_edge(edge,sample_num):
    sample_points = []
    vertice1 = edge.vertices[1]
    interval_length = ((edge.vertices[0][0]-edge.vertices[1][0])**2 +(edge.vertices[0][1]-edge.vertices[1][1])**2 )**0.5/(sample_num-1)
    if interval_length != 0:
        for i in range(sample_num):
            point = vertice1 + i*edge.edge*interval_length/((edge.vertices[0][0]-edge.vertices[1][0])**2 +(edge.vertices[0][1]-edge.vertices[1][1])**2 )**0.5
            sample_points.append(point)
        return sample_points
    else:
        sample_points = [edge.vertices[0]]
        return sample_points

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


def point_in_polygon(point,polygon):

    inside = False
    x, y = point[0],point[1]
    for edge in polygon.edge:
        p1 = edge.vertices[0]
        p2 = edge.vertices[1]
        if (x == p1[0] and y == p1[1]) or (x == p2[0] and y == p2[1]):
            return True
        # Check if the point is above the minimum y coordinate of the edge

        if y > min(p1[1], p2[1]):
            # Check if the point is below the maximum y coordinate of the edge
            if y <= max(p1[1], p2[1]):
                # Check if the point is to the left of the maximum x coordinate of the edge
                if x <= max(p1[0], p2[0]):
                
                    # Calculate the x-intersection of the line connecting the point to the edge
                    x_intersection = (y - p1[1]) * (p2[0] - p1[0]) / (p2[1] - p1[1]) + p1[0]

                    # Check if the point is on the same line as the edge or to the left of the x-intersection
                    if p1[0] == p2[0] or x <= x_intersection:
                        # Flip the inside flag
                        inside = not inside
                
    # Return the value of the inside flag
    return inside

def rotate(theta,vertice):
    rotational_matrix = np.array(
        [np.array([math.cos(theta),-math.sin(theta)]),
        np.array([math.sin(theta),math.cos(theta)])]
    )
    rotated_vertice = rotational_matrix@np.array(vertice).T
    return rotated_vertice
        
    

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
    

def dijkstra(graph, start_index, end_index):
    
    closed_list = []
    final_cost_list = []
    final_path_list = []
    open_list = []
    cost_list = []
    path_list = []
    current_node = graph[start_index]
    closed_list.append(start_index)
    final_cost_list.append(0)
    final_path_list.append([start_index])
    for next_index in current_node.child:
        if not next_index in closed_list:
            open_list.append(next_index)
            cost = ((graph[next_index].x-current_node.x)**2 + (graph[next_index].y-current_node.y)**2)**0.5 + final_cost_list[-1]
            path = copy.deepcopy(final_path_list[-1])
            path.append(next_index)
            cost_list.append(cost)
            path_list.append(path)
            
    while open_list:
        res = np.where(cost_list == np.min(cost_list))[0]
        index = open_list.pop(res[0])
        cost = cost_list.pop(res[0])
        path = path_list.pop(res[0])
        closed_list.append(index)
        final_cost_list.append(cost)
        final_path_list.append(path)
        if index == end_index:
            return path
        current_node = graph[index]
        for next_index in current_node.child:
            if not next_index in closed_list:
                if not next_index in open_list:
                    open_list.append(next_index)
                    cost = ((graph[next_index].x-current_node.x)**2 + (graph[next_index].y-current_node.y)**2)**0.5 + final_cost_list[-1]
                    path = copy.deepcopy(final_path_list[-1])
                    path.append(next_index)
                    cost_list.append(cost)
                    path_list.append(path)
                else:
                    i = open_list.index(next_index)
                    cost = ((graph[next_index].x-current_node.x)**2 + (graph[next_index].y-current_node.y)**2)**0.5 + final_cost_list[-1]
                    path = copy.deepcopy(final_path_list[-1])
                    path.append(next_index)
                    if cost < cost_list[i]:
                        cost_list[i] = cost
                        path_list[i] = path
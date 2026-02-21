import numpy as np
import math



class Dubin_path_segment():

    def __init__(self,direction,length):

        self.dir = direction
        self.length = length


def mod_angle(angle,zero2pi = False):   # Convert angle to from -pi to pi
    if not zero2pi:
        mod_angle = (angle + np.pi) % (2 * np.pi) - np.pi
    else:
        mod_angle = (angle) % (2 * np.pi) 
    return mod_angle

def dubin_path_length(path):
    length = 0
    for seg in path:
        length += seg.length
    return length

def dubins_path_planner(initial_pose, final_pose, turning_radius):

    paths = []

    # Get local coordinate

    rotational_matrix = np.array(
        [np.array([math.cos(initial_pose[2]),-math.sin(initial_pose[2])]),
        np.array([math.sin(initial_pose[2]),math.cos(initial_pose[2])])]
    )

    vector =  np.array([final_pose[0]-initial_pose[0],final_pose[1]-initial_pose[1]]).T

    
    local_vector = rotational_matrix.T@vector
    local_angle = final_pose[2] - initial_pose[2]
    local_angle = mod_angle(local_angle)
    d = (local_vector[0]**2 + local_vector[1]**2)**0.5/turning_radius
    theta = mod_angle(math.atan2(local_vector[1],local_vector[0]))
    a = mod_angle(-theta)
    b = mod_angle(local_angle-theta)
    
    # LSL
    
    
    LSL = []

    c = 2 + d**2-(2*math.cos(a-b)) + (2*d*(math.sin(a) - math.sin(b)))

    if c >= 0: 
        temp = math.atan2(math.cos(b) - math.cos(a), d + math.sin(a) - math.sin(b))
        d1 = mod_angle(temp-a, True)
        L_seg = Dubin_path_segment(-1,abs(d1*turning_radius))
        LSL.append(L_seg)

        d2 = c**0.5
        S_seg = Dubin_path_segment(0,abs(d2*turning_radius))
        LSL.append(S_seg)

        d3 = mod_angle(b - temp, True)
        L_seg = Dubin_path_segment(-1,abs(d3*turning_radius))
        LSL.append(L_seg)

        paths.append(LSL)
    
    # LSR

    LSR = []

    c = -2 + d**2 + (2*math.cos(a-b)) + (2*d*(math.sin(a) + math.sin(b)))

    if c >= 0: 
        d2 = c**0.5
        temp = math.atan2((-math.cos(a) - math.cos(b)), (d + math.sin(a) + math.sin(b))) - math.atan2(-2, d2)
        d1 = mod_angle(temp-a, True)
        L_seg = Dubin_path_segment(-1,abs(d1*turning_radius))
        LSR.append(L_seg)
               
        S_seg = Dubin_path_segment(0,abs(d2*turning_radius))
        LSR.append(S_seg)

        d3 = mod_angle(temp - mod_angle(b), True)
        R_seg = Dubin_path_segment(1,abs(d3*turning_radius))
        LSR.append(R_seg)

        paths.append(LSR)

    # RSR

    RSR = []

    c = 2 + d**2 - (2*math.cos(a-b)) + (2*d*(math.sin(b) - math.sin(a)))

    if c >= 0:
        temp = math.atan2((math.cos(a) - math.cos(b)), d - math.sin(a) + math.sin(b))
        d1 = mod_angle(a - temp,True)
        R_seg = Dubin_path_segment(1,abs(d1*turning_radius))
        RSR.append(R_seg)


        d2 = c**0.5
        S_seg = Dubin_path_segment(0,abs(d2*turning_radius))
        RSR.append(S_seg)
        
        d3 = mod_angle(-b + temp, True)
        R_seg = Dubin_path_segment(1,abs(d3*turning_radius))
        RSR.append(R_seg)

        paths.append(RSR)

    # RSL

    RSL = []

    c = d**2 - 2 + (2*math.cos(a-b)) - (2*d*(math.sin(a) + math.sin(b)))

    if c >= 0:
        d2 = c**0.5
        temp = math.atan2(math.cos(a) + math.cos(b), d - math.sin(a) - math.sin(b)) - math.atan2(2, d2)
        d1 = mod_angle(a - temp,True)
        R_seg = Dubin_path_segment(1,abs(d1*turning_radius))
        RSL.append(R_seg)

        S_seg = Dubin_path_segment(0,abs(d2*turning_radius))
        RSL.append(S_seg)

        d3 = mod_angle(b - temp,True)
        L_seg = Dubin_path_segment(-1,abs(d3*turning_radius))
        RSL.append(L_seg)

        paths.append(RSL)

    # RLR

    RLR = []

    temp = (6 - d**2 + 2*math.cos(a-b) + 2*d*(math.sin(a) - math.sin(b)))/8
    if abs(temp) <= 1:
        d2 = mod_angle(2*math.pi - math.acos(temp),True)
        d1 = mod_angle(a - math.atan2(math.cos(a) - math.cos(b), d - math.sin(a) + math.sin(b)) + d2/ 2,True)
        R_seg = Dubin_path_segment(1,abs(d1*turning_radius))
        RLR.append(R_seg)

        L_seg = Dubin_path_segment(-1,abs(d2*turning_radius))
        RLR.append(L_seg)

        d3 = mod_angle(a - b - d1 + d2,True)
        R_seg = Dubin_path_segment(1,abs(d3*turning_radius))
        RLR.append(R_seg)

        paths.append(RLR)

    # LRL

    LRL = []

    temp = (6 - d**2 + 2*math.cos(a-b) + 2*d * (- math.sin(a) + math.sin(b)))/8

    if abs(temp) <= 1:
        d2 = mod_angle(2*math.pi - math.acos(temp),True)
        d1 = mod_angle(-a - math.atan2(math.cos(a) - math.cos(b), d + math.sin(a) - math.sin(b)) + d2/2,True)
        
        L_seg = Dubin_path_segment(-1,abs(d1*turning_radius))
        LRL.append(L_seg)

        R_seg = Dubin_path_segment(1,abs(d2*turning_radius))
        LRL.append(R_seg)

        d3 = mod_angle(mod_angle(b) - a - d1 + mod_angle(d2),True)
        L_seg = Dubin_path_segment(-1,abs(d3*turning_radius))
        LRL.append(L_seg)

        paths.append(LRL)
    return paths


def cross_product(x,y):
    return x[0]*y[1]-x[1]*y[0]


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
    

def sample_path(path, initial_pose, interval, turning_rad):
        
    pose_sample = []
    current_pose = initial_pose
    pose_sample.append(current_pose)
    remaining_length = 0
    for seg in path:
        
        left_over_interval = interval - remaining_length

        if left_over_interval < seg.length:
            direction = seg.dir
            left_over_seg = Dubin_path_segment(direction, left_over_interval)
            current_pose = get_current_pose(left_over_seg, current_pose, turning_rad)
            pose_sample.append(current_pose)
            remaining_length = seg.length - left_over_interval
            while remaining_length >= interval:
                interval_seg = Dubin_path_segment(direction, interval)
                current_pose = get_current_pose(interval_seg,current_pose,turning_rad)
                pose_sample.append(current_pose)
                remaining_length -= interval
            
            left_over_seg = Dubin_path_segment(direction, remaining_length)
            current_pose = get_current_pose(left_over_seg, current_pose, turning_rad)

        else:
            direction = seg.dir
            current_pose = get_current_pose(seg, current_pose, turning_rad)
            remaining_length += seg.length

    left_over_interval = interval - remaining_length
    left_over_seg = Dubin_path_segment(direction, left_over_interval)
    current_pose = get_current_pose(left_over_seg, current_pose, turning_rad)
    pose_sample.append(current_pose)

    return pose_sample
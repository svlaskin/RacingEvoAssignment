#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# -*- coding: utf-8 -*-

import numpy as np
import time

"""
------------ Agent Constants Definition ---------------------------------------
Here, the constants (such as the car mass) will be defined
"""
# Car agent constants - assuming Porsche 911 GT3
max_lat_g = 12 # 1.24 g (12 m/s²)
wheelbase = 2.46 # [m]
max_steer_angle = 600
w = 47 # deg/s

# Assumed values
time_full_steer = 0.07 # time to fully steer left to right, [s]
min_r_steer = 11 # min turn radius, [m]
a_max = 5 # [m/s2] maximum accelleration (assumed linear)
d_max = 8 # [m/s2] maximum decelaration (assumed linear)
spd_turn = 7 # [m/s] speed where none of the turns are an issue
spd_max = 80


# Car agent initial conditions
steer_angle = 0.

# Derive steering input range
max_steer_left = max_steer_angle/2 # in one direction
max_alpha_f = np.arcsin(wheelbase/min_r_steer) # max sideways front wheel rotation under steering

# time sim parameter dt
dt = 0.01 # [s]


"""
---------- Agent Definition ---------------------------------------------------
Here, the car agent is defined, with its reactive characteristics.
"""

def car_agent(v_car, pos_car, dis2wall_r, dis2wall_l, dis2centerline, center_curve, hdg_nxt, genome):
    """
    ------------- Read Genome ------------------------------------------------
    """
    [steer_const, dis2center_tol, heading_delta_tol, future2add, indfuture, acc_const]=genome
    
    
    """
    --------- Necessary processing for inputs ---------------------------------
    Here, several parameters are computed from the inputs to allow for the agent 
    to make decisions about heading and speed adjustments.
    """
    # delta_dist allows to check whether the car is closer to left or right track boundary
    delta_dist = dis2wall_r-dis2wall_l
    vec_to_center_all = center_curve-pos_car # this is a vector of the car to all points in the centerline
    squared_vec = vec_to_center_all**2 
    dis_squared = squared_vec[:,0]+squared_vec[:,1]
    dis_to_center_all = dis_squared**0.5 # square root to find the distance to all points
    index_closest = (np.where(dis_to_center_all==np.min(dis_to_center_all)))[0][0] # find closest index
    nearest_centerpoint = center_curve[index_closest] # nearest point on centerline
    index_future = int((index_closest+np.shape(np.shape(hdg_nxt))[0]+indfuture)%np.shape(hdg_nxt)[0])
    index_past = (index_closest+np.shape(np.shape(hdg_nxt))[0]-1)%np.shape(hdg_nxt)[0]
    index_future2 = int((index_closest+np.shape(np.shape(hdg_nxt))[0]+future2add)%np.shape(hdg_nxt)[0]) # for far ahead
    heading_next = (hdg_nxt[index_future])
    heading_future = hdg_nxt[index_future2] 
    hdg_car = (np.arctan2(v_car[1],v_car[0])*180/np.pi +360)%360
    spd_car = np.sqrt(v_car[1]**2+v_car[0]**2)
    heading_next = (heading_next*180/np.pi +360)%360 # heading_next is the heading to the next centerpoint.
    heading_delta = (hdg_car-heading_next)
    omega_max = (max_lat_g/spd_car)*180/np.pi # maximum angular velocity from circular motion, a/v [rad/s]
    
    heading_delta = hdg_car-heading_next
    
    set_hdg = hdg_car
    
    """
    ----------- Main Controller -----------------------------------------------
    This section uses information such as the location and heading to points on the centerline
    and distance to the left and right walls to decide on the car agent's 
    reaction (change in steering or throttle/brake input)
    """
    
    # This section controls the heading of the car based on wall distances
    if dis2wall_l>dis2wall_r: # on right side
        if dis2centerline<dis2center_tol:
            set_hdg = heading_next if np.abs(hdg_car-heading_next)<omega_max*dt else hdg_car+steer_const*omega_max*dt
        else:
            set_hdg = hdg_car+omega_max*dt
                
    if dis2wall_l<dis2wall_r: # on left side
        if dis2centerline<dis2center_tol:
            set_hdg = heading_next if np.abs(hdg_car-heading_next)<omega_max*dt else hdg_car-steer_const*omega_max*dt
        else:
            set_hdg = hdg_car-omega_max*dt
    

    
    # here, we modulate the speed of the car
    if np.abs(heading_future-heading_next)>heading_delta_tol and spd_car>spd_turn: # anticipating a large turn, we slow down as fast as possible
        spd_car=spd_car-d_max*dt
            
    elif np.abs(heading_future-heading_next)<=15: # if small deviation, then accellerate lightly
        if spd_car<spd_max: # leave a buffer
            spd_car=spd_car+a_max*dt*acc_const
    
    elif np.abs(heading_future-heading_next)<5:
        if spd_car<spd_max:
            spd_car=spd_car+a_max*dt
    
    # finally, convert everything back to velocity    
    hdg_car = set_hdg # set heading and convert to radians
    v_car = np.array([spd_car*np.cos(np.radians(hdg_car)),spd_car*np.sin(np.radians(hdg_car))]) # update velocity vector 
    
    # print("nxt ", heading_next, "car ", hdg_car)
    return v_car, nearest_centerpoint, index_closest, spd_car, hdg_car
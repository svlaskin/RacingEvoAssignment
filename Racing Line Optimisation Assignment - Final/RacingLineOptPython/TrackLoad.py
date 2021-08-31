#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jul 17 12:36:08 2021

@author: Sasha
"""

## Imports here
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


"""
----------------------- Import Track Data -------------------------------------
Define the path and open file as Pandas DataFrame
"""
# Path to all tracks
fpath_tracks = "/Users/Sasha/Desktop/Racing Line Optimisation Assignment/Racetrack_db/tracks"

# Zandvoort
example_track = "/Users/Sasha/Desktop/Racing Line Optimisation Assignment/Racetrack_db/tracks/Zandvoort.csv"
zandvoort_db = pd.read_csv(example_track) # read file

"""
----------------------- Process data func ------------------------------------------
Convert necesary DataFrame rows to arrays, and compute the left and right
boundary point positions
"""

def track_process(track_db):
    # Centerline
    x_c = np.array(zandvoort_db.x_m)
    y_c = np.array(zandvoort_db.y_m)
    
    # Left Boundary - from center in meters
    w_l = np.array(zandvoort_db.w_tr_left_m)
    
    # Right Boundary - from center in meters
    w_r = np.array(zandvoort_db.w_tr_right_m)
    
    # ----- Compute the left and right boundaries as coordinates
    x_c_forward = np.append([x_c[-1]],x_c[0:-1]) # shift array 1 point forward
    y_c_forward = np.append([y_c[-1]],y_c[0:-1]) # same
    
    # use these to compute the direction/heading between each point and the next
    delta_x = -(x_c_forward-x_c) # y coor change from current point to next
    delta_y = -(y_c_forward-y_c) # x coor change from current point to next
    hdg_to_nxt = np.arctan2(delta_y, delta_x) # heading to next point in radians
    
    # heading offset for right and left boundary
    hdg_right = hdg_to_nxt - np.pi/2
    hdg_left = hdg_to_nxt + np.pi/2
    
    # comput the coordinate using angle and width (magnitude)
    delta_x_l = w_l*np.cos(hdg_left)
    delta_y_l = w_l*np.sin(hdg_left)
    delta_x_r = w_r*np.cos(hdg_right)
    delta_y_r = w_r*np.sin(hdg_right)
    
    x_l = x_c+delta_x_l
    y_l = y_c+delta_y_l
    x_r = x_c+delta_x_r
    y_r = y_c+delta_y_r

    return x_c, y_c, x_l, y_l, x_r, y_r, hdg_to_nxt, w_l, w_r

def plot_track(x_c, y_c, x_l, y_l, x_r, y_r):
    """
    ------------------ Plot The centerline and boundaries ------------------------
    Plot the centerline using the coordinates given in the csv and the computed
    left and right track limits
    """
    
    # Set labels
    plt.xlabel("East in m")
    plt.ylabel("North in m")
    
    # Plot centerline in red
    plt.plot(x_c,y_c,'r')
    
    # plt.plot(x_c[0:200],y_c[0:200],'c') # direction test
    
    # Plot the left and right boundaries in blue
    plt.plot(x_l,y_l,'blue')
    plt.plot(x_r,y_r,'green')
    
    plt.show()
    
    return

def plot_result(x_c, y_c, x_l, y_l, x_r, y_r, lap_trace):
    """
    ------------------ Plot The track and Car Trajecotry------------------------
    Plot the centerline using the coordinates given in the csv and the computed
    left and right track limits. Then plot car traj in green
    """
    
    # Set labels
    plt.xlabel("East in m")
    plt.ylabel("North in m")
    
    # # Plot centerline in orange
    plt.plot(x_c,y_c,'orange')
    
    # plt.plot(x_c[0:200],y_c[0:200],'c') # direction test
    
    # Plot the left and right boundaries in blue
    plt.plot(x_l,y_l,'yellow')
    plt.plot(x_r,y_r,'teal')
    
    # Plot the car trajectory
    lap_trace = np.array(lap_trace)
    plt.plot(lap_trace[:,[0]], lap_trace[:,[1]] ,'r')
    plt.show()
    print("Result Plotted")
    return

def random_scaled(maxval, minval, rand):
    return minval + (rand * (maxval - minval))

# some test code
# x_c, y_c, x_l, y_l, x_r, y_r, hdg_to_nxt = track_process(zandvoort_db)
# plot_track(x_c, y_c, x_l, y_l, x_r, y_r)



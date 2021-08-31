#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This file was usef to run the best genome again and obtain telemetry such as the
velocity profile, as well as plot the gene graphs VS generation once
RacingLineOpt.py had been run.

@author: Sasha
"""
## Imports here
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from TrackLoad import *
from AgentEvo import *
import shapely.geometry as geom
import time as perftimer
from time import perf_counter
from random import seed
from random import random
from random import randint
import pandas as pd

"""
------------------ Import Track Data ------------------------------------------
"""
# Path to track .csv file
fpath_tracks = "/Users/Sasha/Desktop/Racing Line Optimisation Assignment/Racetrack_db/tracks"
# Zandvoort
example_track = "/Users/Sasha/Desktop/Racing Line Optimisation Assignment/Racetrack_db/tracks/Zandvoort.csv"
zandvoort_db = pd.read_csv(example_track) # read file
# Import track geometry (centerline and boundaries)
x_c, y_c, x_l, y_l, x_r, y_r, hdg_nxt, w_l, w_r = track_process(zandvoort_db)
# Make coordinate sets out of centerline, left and right bounds
left_curve = list(zip(x_l,y_l))
right_curve = list(zip(x_r,y_r))
center_curve = list(zip(x_c,y_c))
# Convert to shapely objects (for distance computation)
center_line = geom.LineString(center_curve)
left_line = geom.LineString(left_curve)
right_line = geom.LineString(right_curve)

genome_list = [[0.9646288007301632,
 1.1315077291833093,
 82.74896339148233,
 5,
 0,
 0.7977668586353102]] # Best genome obtained from run C

t_start = perftimer.perf_counter()
i=0
for genome in genome_list:
    lap_trace = []
    v_trace = []
    hdg_inp_trace = []
    time_vec = []
    """------ Initial Conditions -----------------------------------------------"""
    x_car, y_car = x_c[0], y_c[0] # Init position(on center of start line) [m]
    heading_car = hdg_nxt[0] # Init heading [rad]
    spd_car = 7 # inital speed [m/s], CURRENTLY LEFT CONSTANT
    # vectorise this for later
    pos_car = np.array([x_car,y_car])
    v_car = np.array([spd_car*np.cos(heading_car),spd_car*np.sin(heading_car)])
    finished = False # condition for car having completed the track
    crashed = False # condition if car is out of track bounds
    time_sim=0
    i=i+1
    print("doing ", i, " from ", np.shape(genome_list)[0])
    while not finished and time_sim <800 and not crashed: # conditions for termination
        
        # add increment to time
        time_sim += dt
        time_vec.append(time_sim)
        # Compute the "vision" components of the car agents, namely the distance to
        # left and right walls and the itnersection point with the track ahead.
        dis2wall_r = np.abs(geom.Point(pos_car).distance(right_line))
        dis2wall_l = np.abs(geom.Point(pos_car).distance(left_line))
        dis2centerline = np.abs(geom.Point(pos_car).distance(center_line))
        
        genome_current = genome
        
        # Feed current parameters into car to get velocity update
        v_car, nearest_centerpoint, index_closest, spd_car, hdg_car = car_agent(v_car, pos_car, dis2wall_r, dis2wall_l, dis2centerline, center_curve, hdg_nxt, genome_current)
        v_trace.append(spd_car)
        hdg_inp_trace.append(hdg_car)
        # Compute next position with new velocity
        pos_car = pos_car + v_car*dt
        lap_trace.append(pos_car)
        
        # Car reaches finish line when the nearest point is the start/finish line point
        if nearest_centerpoint == center_curve[0] and time_sim>50:
            finished = True
        # check if crashed: this happens when distance to centerline in either direction exceeds track width  
        if dis2centerline > w_r[index_closest]-0.1 or dis2centerline > w_l[index_closest]-0.1:
            crashed=True
    if not crashed:
            lap_trace_fastest = lap_trace
            plot_result(x_c, y_c, x_l, y_l, x_r, y_r, lap_trace_fastest)
            
# Some extra plot stuff
plt.plot(plot_genome[:,[0]])
plt.xlabel("Generation")
plt.ylabel("Steer Const. [-]")
plt.grid()
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 23 23:44:48 2022

@author: lakij
"""

import pickle
from EggbotControl import EggbotControl
import numpy as np
import time

def find_max_min(data, color_order):
    minx = 1e10
    miny = 1e10
    maxx = -1e10
    maxy = -1e10
    for i in color_order:
        for cnt in data[i]:
            minx = min(np.min(np.array(cnt)[:,0]),minx)
            miny = min(np.min(np.array(cnt)[:,1]),miny)
            maxx = max(np.max(np.array(cnt)[:,0]),maxx)
            maxy = max(np.max(np.array(cnt)[:,1]),maxy)
    return minx, miny, maxx, maxy

maxx = 3600
maxy = 850
color_order = ['yellow', 'cyan', 'gray', 'orange', 'green', 'red', 'brown', 'blue', 'black']
move_speed = 1000
draw_speed = 600
drop_delay = 0.1
raise_delay = 0.2

port = "COM7"
infile = r"C:\Users\lakij\Documents\eggbot\eggbot_SW\contour_files\circles_wrap_cnts.pickle"



ctrl = EggbotControl(port)
with open(infile, 'rb') as f:
    data = pickle.load(f)
    
ctrl.lift_servo()
ctrl.zero_steppers()
maxx, maxy = ctrl.get_max_position()

stepperx = 0
steppery = 0
    
for color in color_order:
    ctrl.lift_servo()
    num_cnt = len(data[color])
    if num_cnt == 0:
        continue
    stepperx, steppery = ctrl.get_position()
    ctrl.set_speed(move_speed, move_speed)
    ctrl.move(0, maxy // 2 - steppery)
    
    print("Insert",color," marker and then press enter", end = "")
    input()
    for icnt, cnt in enumerate(data[color]):
        print("Drawing contour",icnt+1,"of",num_cnt)
        first_point = 1
        for curx,cury in cnt:
            stepperx, steppery = ctrl.get_position()
            if first_point:
                ctrl.set_speed(move_speed, move_speed)
                pass
            else:
                ctrl.set_speed(draw_speed, draw_speed)
                pass
            x_rel = curx - stepperx
            y_rel = cury - steppery
            if (x_rel > maxx/2):
                x_rel = x_rel - maxx
            elif (x_rel < -maxx/2):
                x_rel = x_rel + maxx
            
            ctrl.move(x_rel, y_rel)
            if first_point:
                ctrl.lower_servo()
                time.sleep(drop_delay)
            stepperx, steppery = curx, cury
            first_point = 0
        ctrl.lift_servo()
        time.sleep(raise_delay)
    
    print("Done drawing")
        
        
    
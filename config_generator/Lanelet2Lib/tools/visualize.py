

import matplotlib.pyplot as plt
import numpy as np
import math
import subprocess

car_len = 4
car_width = 2


ego_car = []
special_point2 = []
all_cars = []

x_points_l = []
y_points_l = []
x_points_m = []
y_points_m = []
x_points_r = []
y_points_r = []

init_x = 561364.293
init_y = 4194837.207


def read_road_nodes(x_points, y_points, path):
	with open(path) as f:
		for line in f.readlines():
			x, y = [float(y) for y in line.strip('\n').strip().split(",")]
			x = x - init_x
			y = y - init_y
			x_points.append(x)
			y_points.append(y)

def draw_road(m_ax):
	m_ax.plot(x_points_m, y_points_m, color='black')
	m_ax.plot(x_points_l, y_points_l, color='black')
	m_ax.plot(x_points_r, y_points_r, color='black')

def find_postion(x, x_points, y_points):
	
	res = [0, 0] #TODO
	l = np.zeros(len(x_points))
	for i in range(len(x_points) - 1):
		l[i+1] = l[i] + math.sqrt(pow(x_points[i+1] - x_points[i], 2) + pow(y_points[i+1] - y_points[i], 2))
		if (x > l[i] and x < l[i+1]):
			res = [(x_points[i+1]-x_points[i])*(x-l[i])/(l[i+1] - l[i]) + x_points[i], ((y_points[i+1]-y_points[i])*(x-l[i])/(l[i+1] - l[i]) + y_points[i])]
			if(res == None):
				print(x)
				print(l[i])
				print(l[i+1])
	return res
img_id = 0;
with open('txt') as f:
    for line in f.readlines():
        if("---" in line):
            plt.cla()
            fig = plt.figure(num=1, figsize=(18,8))
            ax = fig.add_subplot(1, 1, 1)
            # ax.figure.set_size_inches(80, 60)
            draw_road(ax)
            ax.set_title('Simulator', fontsize=12, color = 'g')
            ax.set_xlabel('X in UTM')
            ax.set_ylabel('Y in UTM')
            ax.set_xlim(-100,400)
            ax.set_ylim(-200,200)
            for car in all_cars:
                clr = 'violet'
                agl = car[3]*180/(math.pi)
              #  ax.add_patch(plt.Rectangle((car[1] - car_width/2, car[2]-car_len/2), car_width, car_len, angle=agl, fill=True, color=clr, linewidth=1))
                ax.add_patch(plt.Rectangle((car[1], car[2]), car_len, car_width, angle=agl, fill=True, color=clr, linewidth=1))
            plt.savefig("image_%02d.png" %img_id)
            img_id = img_id + 1
            all_cars.clear()
            plt.pause(0.001)
            continue
           
        tmp = [float(y) for y in line.strip('\n').strip().split(",")]
        all_cars.append(tmp)
#        print(all_cars)
subprocess.call([
     'ffmpeg', '-framerate', '8', '-i', 'image_%02d.png', '-r', '30', '-pix_fmt', 'yuv420p',
     'stop_with_idm.mp4'
 ])

#!/usr/bin/env python
import lanelet2
import matplotlib
import matplotlib.axes
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from math import cos, sin, sqrt, pow
import scipy.io
import os, select, termios, tty, sys


msg = """
Please choose the map!
---------------------------

        '0':	"DR_CHN_Merging_ZS"
        '1':	"DR_CHN_Roundabout_LN"
        '2':	"DR_DEU_Merging_MT"
        '3':	"DR_DEU_Roundabout_OF"
        '4':	"DR_USA_Intersection_EP0"
        '5':	"DR_USA_Intersection_EP1"
        '6':	"DR_USA_Intersection_GL"
        '7':	"DR_USA_Intersection_MA"
        '8':	"DR_USA_Roundabout_EP"
        '9':	"DR_USA_Roundabout_FT"
        '.':	"TC_Intersection_VA"
        '+':	"DR_USA_Roundabout_SR"

CTRL-C to quit
"""

maplib = {
        '0':"DR_CHN_Merging_ZS",
        '1':"DR_CHN_Roundabout_LN",
        '2':"DR_DEU_Merging_MT",
        '3':"DR_DEU_Roundabout_OF",
        '4':"DR_USA_Intersection_EP0",
        '5':"DR_USA_Intersection_EP1",
        '6':"DR_USA_Intersection_GL",
        '7':"DR_USA_Intersection_MA",
        '8':"DR_USA_Roundabout_EP",
        '9':"DR_USA_Roundabout_FT",
        '.':"TC_Intersection_VA",
        '+':"DR_USA_Roundabout_SR",
    }


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
    

def plotter():
    rospy.init_node('map_roundabout', anonymous=True)
    pub = rospy.Publisher("map_roundabout_map", MarkerArray, queue_size=1)
    rate = rospy.Rate(20)
    close_flag = 1
    print(msg)

    while(1):
        key = getKey()
        if key in maplib.keys():
            map_name = maplib[key]
            show = "plotting: "+map_name
            print(show)
            lanelet_map_file = os.path.dirname(__file__) + "/with_negative_xy/"+map_name+".osm"
            lat_origin = 0.  # origin is necessary to correctly project the lat lon values in the osm file to the local
            lon_origin = 0.  # coordinates in which the tracks are provided; we decided to use (0|0) for every scenario
            projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(lat_origin, lon_origin))
            laneletmap = lanelet2.io.load(lanelet_map_file, projector)
            break
        else:
            if key =='\x03':
                close_flag = 0
                break
            print("You just give a wrong number. Please choose the map form this list.") 
            print(msg)



    while (rospy.is_shutdown() != 1 and close_flag != 0):
        vehi = MarkerArray()
        i = 0
        for ls in laneletmap.lineStringLayer:
            veh = Marker()
            veh.header.frame_id = "/my_frame"
            veh.header.stamp = rospy.get_rostime()
            veh.type = veh.LINE_STRIP
            veh.ns = "mkzvehicle"
            veh.scale.x = 0.15
            veh.color.b = 1.0
            veh.color.r = 1.0
            veh.color.g = 1.0
            veh.color.a = 1.0
            veh.action = veh.ADD
            veh.id = i
            i = i +1
            if "type" not in ls.attributes.keys():
                raise RuntimeError("ID " + str(ls.id) + ": Linestring type must be specified")

            elif ls.attributes["type"] == "curbstone":
                veh.color.b = 1.0
                veh.color.r = 1.0
                veh.color.g = 1.0

            elif ls.attributes["type"] == "guard_rail":
                veh.color.b = 1.0
                veh.color.r = 1.0
                veh.color.g = 1.0

            elif ls.attributes["type"] == "line_thin":
                if "subtype" in ls.attributes.keys() and ls.attributes["subtype"] == "dashed":
                    veh.color.b = 0.0
                    veh.color.r = 1.0
                    veh.color.g = 1.0
                    veh.color.a = 0.3
                else:
                    veh.color.b = 0.0
                    veh.color.r = 1.0
                    veh.color.g = 1.0
            elif ls.attributes["type"] == "line_thick":
                if "subtype" in ls.attributes.keys() and ls.attributes["subtype"] == "dashed":
                    veh.scale.x = 0.25
                    veh.color.r = 1.0
                    veh.color.g = 1.0
                    veh.color.a = 0.3
                else:
                    veh.scale.x = 0.25
                    veh.color.r = 1.0
                    veh.color.g = 1.0
            elif ls.attributes["type"] == "pedestrian_marking":
                veh.color.b = 0.0
                veh.color.r = 0.0
                veh.color.g = 1.0
            elif ls.attributes["type"] == "bike_marking":
                veh.color.b = 0.0
                veh.color.r = 0.0
                veh.color.g = 1.0
            elif ls.attributes["type"] == "stop_line" or ls.attributes["type"] == "traffic_sign":
                veh.color.b = 0.0
                veh.color.r = 1.0
                veh.color.g = 0.0
                veh.scale.x = 0.25
            elif ls.attributes["type"] == "virtual":
                veh.color.b = 1.0
                veh.color.r = 1.0
                veh.color.g = 0.0
            elif ls.attributes["type"] == "road_border":
                veh.color.b = 1.0
                veh.color.r = 1.0
                veh.color.g = 1.0
            else:
                raise RuntimeError("ID " + str(ls.id) + ": Linestring type not known")
            for pt in ls:
                p = Point()
                p.x = pt.x
                p.y = pt.y
                p.z = 0
                veh.points.append(p)
            vehi.markers.append(veh)
        pub.publish(vehi)

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    plotter()

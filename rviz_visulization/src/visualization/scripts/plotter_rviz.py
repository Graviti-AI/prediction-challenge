#!/usr/bin/env python
import rospy
import scipy.io
import os, rospkg
from visualization.msg import state_Dynamic, Trajectory2D, TrajectoryPoint2D, vehicleinfo, vehicleinfomation
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Header, Float64MultiArray
from geometry_msgs.msg import Point
from math import cos, sin, sqrt, pow
from std_msgs.msg import String, Float64
#####

ini_flag = False
ini_flag2 = False
ini_flag3 = False
ini_flag4 = False
ini_flag5 = False
ini_flag6 = False
ini_flag7 = False
X2 = [0.0]
Y2 = [0.0]
X3 = [0.0]
Y3 = [0.0]
X6 = [0.0]
Y6 = [0.0]
lx = [0.0]
ly = [0.0]
angle = [0]
number = 0

X_y = [0.0]   #yielding situation
Y_y = [0.0]
Theta_y = [0.0]
Theta2 = [0.0]
ini_flag_Y = False


## State_estimate information callback
def callback(data):
    global ini_flag, X, Y, psi
    X = data.X
    Y = data.Y
    psi = data.psi
    ini_flag = True

## final_trajectory_information callback

def ref_traj_callback(data) :
    global ini_flag2, X2, Y2, Theta2
    X2tem = []
    Y2tem = []
    Theta2tem = []
    ini_flag2 = True
    for index in range(len(data.point)):
        X2tem.append(data.point[index].x)
        Y2tem.append(data.point[index].y)
        Theta2tem.append(data.point[index].theta)
    X2 = X2tem
    Y2 = Y2tem
    Theta2 = Theta2tem

def ref_traj_callback2(data) :
    global ini_flag_Y, X_y, Y_y, Theta_y
    X_ytem = []
    Y_ytem = []
    Theta_ytem = []
    ini_flag_Y = True
    for index in range(len(data.point)):
        X_ytem.append(data.point[index].x)
        Y_ytem.append(data.point[index].y)
        Theta_ytem.append(data.point[index].theta)
    X_y = X_ytem
    Y_y = Y_ytem
    Theta_y = Theta_ytem
## vehicle_state callback
def vcallback(data) :
    global ini_flag3, X3, Y3, lx, ly, angle, number
    X3tem = []
    Y3tem = []
    lxtem = []
    lytem = []
    angletem = []
    ini_flag3 = True
    number = len(data.point)
    for inde in range(number):
        X3tem.append(data.point[inde].X)
        Y3tem.append(data.point[inde].Y)
        lxtem.append(data.point[inde].lx)
        lytem.append(data.point[inde].ly)
        angletem.append(data.point[inde].psi)
    X3 = X3tem
    Y3 = Y3tem
    lx = lxtem
    ly = lytem
    angle = angletem

## vehicle1 from simulator information callback
def vehicle1_info_callback(data):
    global  ini_flag4, X4, Y4, psi_4
    X4 = data.X
    Y4 = data.Y
    psi_4 = data.psi
    ini_flag4 = True

## vehicle2 from simulator information callback
def vehicle2_info_callback(data):
    global  ini_flag5, X4_2, Y4_2, psi_4_2
    X4_2 = data.X
    Y4_2 = data.Y
    psi_4_2 = data.psi
    ini_flag5 = True

def Pe_callback(data):
    global ini_flag6, X5, Y5, psi_5
    X5 = data.data[0]
    Y5 = data.data[1]
    psi_5= data.data[2]
    ini_flag6 = True

def realref_callback(data):
    global ini_flag7, X6, Y6
    X6tem = []
    Y6tem = []
    #ini_flag7 = True
    for index in range(0, len(data.data), 3):
        X6tem.append(data.data[index])
        Y6tem.append(data.data[index+1])
    Y6 = Y6tem
    X6 = X6tem
    if (len(data.data) == 0):
        ini_flag7 = False



## Visualization the final_trajectory, original map_traj, real_vehicle, simulator_vehicle 1 & 2 in RVIZ by publishing accord topic and adding them in RVIZ
def plotter():
    global ini_flag, ini_flag2,ini_flag_Y, ini_flag3, X, Y, psi, X2, Y2, X3, Y3, lx, ly, angle, number, veh, ini_flag4, X4, Y4, psi_4, ini_flag5, X4_2, Y4_2, psi_4_2, Y6,ini_flag7, X6
    # initialize node
    rospy.init_node('plotter', anonymous=True)
    rospack = rospkg.RosPack()


    rospy.Subscriber('state_estimate', state_Dynamic, callback, queue_size=1)
    rospy.Subscriber('vehicle_state', vehicleinfo, vcallback, queue_size=1)
    rospy.Subscriber('final_trajectory', Trajectory2D, ref_traj_callback, queue_size=1)
    rospy.Subscriber('final_yield_trajectory', Trajectory2D, ref_traj_callback2, queue_size=1)
    #rospy.Subscriber('vehicle_info', String, vehicle_info_callback, queue_size = 1)
    rospy.Subscriber('vehicle1_info', state_Dynamic, vehicle1_info_callback, queue_size = 1)
    rospy.Subscriber('vehicle2_info', state_Dynamic, vehicle2_info_callback, queue_size = 1)
    rospy.Subscriber('Pedestrian', Float64MultiArray, Pe_callback, queue_size = 1)
    rospy.Subscriber('realref_data', Float64MultiArray, realref_callback, queue_size = 1)

    v_pub = rospy.Publisher("vehicle", Marker, queue_size=1)
    l_pub = rospy.Publisher("line", Marker, queue_size=1)
    realref_pub = rospy.Publisher("realref_", Marker, queue_size=1)  
    v_info_1_pub = rospy.Publisher('v_info_1', Marker, queue_size=1)
    v_info_2_pub = rospy.Publisher('v_info_2', Marker, queue_size=1)
    Pe_marker_pub = rospy.Publisher('Pe_info', Marker, queue_size=1)
    pub = rospy.Publisher("veh", MarkerArray, queue_size=1)

    Passing_veh_pub = rospy.Publisher("passing_veh", MarkerArray, queue_size=1)
    Yielding_veh_pub = rospy.Publisher("yielding_veh", MarkerArray, queue_size=1)

    rate = rospy.Rate(20)

    while (rospy.is_shutdown() != 1):
        print ("done")
        vmarker = Marker()
        v_info_1_marker = Marker()
        v_info_2_marker = Marker()
        Pe_marker = Marker()
        lmarker = Marker()
        realrefmarker = Marker()
        point_marker = Marker()

        vmarker.header.stamp = rospy.get_rostime()
        v_info_1_marker.header.stamp = rospy.get_rostime()
        v_info_2_marker.header.stamp = rospy.get_rostime()
        Pe_marker.header.stamp = rospy.get_rostime()
        lmarker.header.stamp = rospy.get_rostime()
        realrefmarker.header.stamp = rospy.get_rostime()

        vmarker.header.frame_id = "/my_frame"
        v_info_1_marker.header.frame_id = "/my_frame"
        v_info_2_marker.header.frame_id = "/my_frame"
        Pe_marker.header.frame_id = "/my_frame"
        lmarker.header.frame_id = "/my_frame"
        realrefmarker.header.frame_id = "/my_frame"

        lmarker.ns = "mkzvehicle"
        realrefmarker.ns = "mkzvehicle"
        vmarker.ns = "mkzvehicle"
        v_info_1_marker.ns = "mkzvehicle"
        v_info_2_marker.ns = "mkzvehicle"
        Pe_marker.ns = "mkzvehicle"

        lmarker.id = 1
        realrefmarker.id = 6
        vmarker.id = 2
        v_info_1_marker.id = 3
        v_info_2_marker.id = 4
        Pe_marker.id = 5

        vmarker.scale.x = 2.85
        vmarker.scale.y = 1.6
        vmarker.scale.z = 1.5
        v_info_1_marker.scale.x = 2.85
        v_info_1_marker.scale.y = 1.6
        v_info_1_marker.scale.z = 1.5
        v_info_2_marker.scale.x = 2.85
        v_info_2_marker.scale.y = 1.6
        v_info_2_marker.scale.z = 1.5
        Pe_marker.scale.x = 2
        Pe_marker.scale.y = 2
        Pe_marker.scale.z = 1.8
        lmarker.scale.x = 0.6
        realrefmarker.scale.x = 0.6

        lmarker.color.g = 1.0
        lmarker.color.a = 1.0
        realrefmarker.color.g = 0.5
        realrefmarker.color.r = 0.5
        realrefmarker.color.a = 1.0
        vmarker.color.r = 1.0
        vmarker.color.a = 1.0
        v_info_1_marker.color.r = 1.0
        v_info_1_marker.color.a = 1.0
        v_info_2_marker.color.r = 1.0
        v_info_2_marker.color.a = 1.0
        Pe_marker.color.r = 1.0
        Pe_marker.color.a = 1.0

        lmarker.type = lmarker.LINE_STRIP
        realrefmarker.type = realrefmarker.LINE_STRIP
        vmarker.type = vmarker.CUBE
        v_info_1_marker.type = v_info_1_marker.CUBE
        v_info_2_marker.type = v_info_2_marker.CUBE
        Pe_marker.type = v_info_2_marker.CUBE

        lmarker.action = lmarker.ADD
        realrefmarker.action = realrefmarker.ADD
        vmarker.action = vmarker.ADD
        v_info_1_marker.action = v_info_1_marker.ADD
        v_info_2_marker.action = v_info_2_marker.ADD
        Pe_marker.action = Pe_marker.ADD

        if ini_flag == True:
            vmarker.pose.position.x = X-558700.0
            vmarker.pose.position.y = Y-4196800.0


            vmarker.pose.position.z = 1.25
            vmarker.pose.orientation.x = 0.0
            vmarker.pose.orientation.y = 0.0
            vmarker.pose.orientation.z = sin(psi/2.0)
            vmarker.pose.orientation.w = cos(psi/2.0)
            v_pub.publish(vmarker)

        if ini_flag4 == True:
            v_info_1_marker.pose.position.x = X4 - 558700.0
            v_info_1_marker.pose.position.y = Y4 - 4196800.0

            v_info_1_marker.pose.position.z = 0.75
            v_info_1_marker.pose.orientation.x = 0.0
            v_info_1_marker.pose.orientation.y = 0.0
            v_info_1_marker.pose.orientation.z = sin(psi_4/2.0)
            v_info_1_marker.pose.orientation.w = cos(psi_4/2.0)
            v_info_1_pub.publish(v_info_1_marker)

        if ini_flag5 == True:
            v_info_2_marker.pose.position.x = X4_2 - 558700.0
            v_info_2_marker.pose.position.y = Y4_2 - 4196800.0

            v_info_2_marker.pose.position.z = 0.75
            v_info_2_marker.pose.orientation.x = 0.0
            v_info_2_marker.pose.orientation.y = 0.0
            v_info_2_marker.pose.orientation.z = sin(psi_4_2/2.0)
            v_info_2_marker.pose.orientation.w = cos(psi_4_2/2.0)
            v_info_2_pub.publish(v_info_2_marker)

        if ini_flag6 == True:
            Pe_marker.pose.position.x = X5 - 558700.0
            Pe_marker.pose.position.y = Y5 - 4196800.0

            Pe_marker.pose.position.z = 0.75
            Pe_marker.pose.orientation.x = 0.0
            Pe_marker.pose.orientation.y = 0.0
            Pe_marker.pose.orientation.z = sin(psi_5/2.0)
            Pe_marker.pose.orientation.w = cos(psi_5/2.0)
            Pe_marker_pub.publish(Pe_marker)

        if ini_flag7 == True:
            length = len(X6)
            for i in range(0, length, 10):
                p = Point()
                p.x = X6[i]-558700.0
                p.y = Y6[i]-4196800.0
                p.z = 0.3

                realrefmarker.points.append(p)
            realref_pub.publish(realrefmarker)



        if ini_flag2 == True:
            length = len(X2)
            veh_passing = MarkerArray()
            for i in range(0, length, 10):
                p = Point()
                p.x = X2[i]-558700.0
                p.y = Y2[i]-4196800.0
                p.z = 0.3
                lmarker.points.append(p)

                veh_passing_i = Marker()
                veh_passing_i.header.frame_id = "/my_frame"
                veh_passing_i.header.stamp = rospy.get_rostime()
                veh_passing_i.type = veh_passing_i.CUBE
                veh_passing_i.ns = "mkzvehicle"
                veh_passing_i.scale.z = 1.5
                veh_passing_i.color.b = 1.0*(length-i*0.75)/length
                veh_passing_i.color.a = 1
                veh_passing_i.color.r = 2.25
                veh_passing_i.color.g = 1.5
                veh_passing_i.pose.position.z = 0.75
                veh_passing_i.pose.orientation.x = 0.0
                veh_passing_i.pose.orientation.y = 0.0
                veh_passing_i.action = veh_passing_i.ADD
                veh_passing_i.id = 4+2*i
                veh_passing_i.scale.x = 2.85
                veh_passing_i.scale.y = 1.6

                veh_passing_i.pose.position.x = X2[i] -558700.0
                veh_passing_i.pose.position.y = Y2[i] -4196800.0

                veh_passing_i.pose.orientation.z = sin(Theta2[i]/2.0)
                veh_passing_i.pose.orientation.w = cos(Theta2[i]/2.0)
                veh_passing.markers.append(veh_passing_i)
            ini_flag2 =  False
            Passing_veh_pub.publish(veh_passing)
            l_pub.publish(lmarker)


        if ini_flag_Y == True:
            length = len(X_y)
            veh_yielding = MarkerArray()
            for i in range(0, length, 10):

                veh_yielding_i = Marker()
                veh_yielding_i.header.frame_id = "/my_frame"
                veh_yielding_i.header.stamp = rospy.get_rostime()
                veh_yielding_i.type = veh_yielding_i.CUBE
                veh_yielding_i.ns = "mkzvehicle"
                veh_yielding_i.scale.z = 1.5
                veh_yielding_i.color.r = 1.0*(length-i*0.75)/length
                veh_yielding_i.color.g = 1.0
                veh_yielding_i.color.a = 1.0
                veh_yielding_i.color.b = 1.5
                veh_yielding_i.pose.position.z = 0.75
                veh_yielding_i.pose.orientation.x = 0.0
                veh_yielding_i.pose.orientation.y = 0.0
                veh_yielding_i.action = veh_yielding_i.ADD
                veh_yielding_i.id = 4+2*i
                veh_yielding_i.scale.x = 2.85
                veh_yielding_i.scale.y = 1.6

                veh_yielding_i.pose.position.x = X_y[i] -558700.0
                veh_yielding_i.pose.position.y = Y_y[i] -4196800.0

                veh_yielding_i.pose.orientation.z = sin(Theta_y[i]/2.0)
                veh_yielding_i.pose.orientation.w = cos(Theta_y[i]/2.0)
                veh_yielding.markers.append(veh_yielding_i)
            ini_flag_Y =  False
            Yielding_veh_pub.publish(veh_yielding)

        if ini_flag3 == True:
            vehi = MarkerArray()
            for i in range(len(X3)):
                veh = Marker()
                veh.header.frame_id = "/my_frame"
                veh.header.stamp = rospy.get_rostime()
                veh.type = veh.CUBE
                veh.ns = "mkzvehicle"
                veh.scale.z = 1.5
                veh.color.b = 1.0
                veh.color.r = 1.0
                veh.color.a = 1.0
                veh.pose.position.z = 0.75
                veh.pose.orientation.x = 0.0
                veh.pose.orientation.y = 0.0
                veh.action = veh.ADD
                veh.id = 4+2*i
                veh.scale.x = lx[i]
                veh.scale.y = ly[i]

                veh.pose.position.x = X3[i]
                veh.pose.position.y = Y3[i]

                veh.pose.orientation.z = sin(angle[i]/2.0)
                veh.pose.orientation.w = cos(angle[i]/2.0)
                vehi.markers.append(veh)

            pub.publish(vehi)
        rate.sleep()

if __name__ == '__main__':
    plotter()

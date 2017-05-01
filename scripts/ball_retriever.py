#!/usr/bin/env python
import sys
import roslib
import rospy
import math
import numpy
import smach
import smach_ros
import cv2
import time
import random
import copy

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion
from math import pi, cos, sin, sqrt, atan2
from random import randint
import numpy as np

#define some global variables for data sharing between states
#TODO: bad design: avoid using global variables
dis = 1
waypoints = []
rn = ""
nodename = ""

#in the search state, crawbot searches for balls
#returns GettingBall state
class Searching(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['foundBall'])
        global nodename
        rospy.loginfo(nodename+":Searching initiation done.")
        global rn

        #define class variables
        self.move_pub = rospy.Publisher(rn + '/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.rate = rospy.Rate(60)

        # A for Avoiding obsticales; B for going to Ball; S for searching for balls
        # priority A > B > S
        self.mode = 'S'

        self.found_ball = False
        self.crawbot_odom = Odometry()
        self.scan = LaserScan()
        self.balls = []
        self.ball_angle_offset = 0

    def execute(self, userdata):
        global nodename
        rospy.loginfo(nodename+":Executing state Searching...")

        #reset class variables
        self.found_ball = False
        self.mode = 'S'
        self.waypoints = []

        self.search()

        global waypoints
        waypoints = self.waypoints
        return 'foundBall'

    #the search method lisens to three topics: odom, laserscan, and camera.
    #stops when a ball is found
    def search(self):
        odom_sub = rospy.Subscriber(rn + "/odom", Odometry, self.odom_callback)
        scan_sub = rospy.Subscriber(rn + "/laser/scan", LaserScan, self.scan_callback)
        image_sub = rospy.Subscriber(rn+ "/camera/image_raw",Image, self.image_callback)
        time.sleep(1)

        #main searching loop
        while not self.found_ball:
            step = Twist()
            step.angular.x = 0
            step.angular.z = 0

            #searching mode: searching
            if self.mode == 'S':
                ranges = list(self.scan.ranges)
                leftmax = max(ranges[:10])
                rightmax = max(ranges[710:])

                step.linear.x = .6

                #angular movement: disabled
                if False:
                    if abs(leftmax - rightmax) > 1:
                        if leftmax > rightmax:
                            step.angular.z = -12 *2*pi/360
                        else:
                            step.angular.z = 12 *2*pi/360

            #searching mode: avoid obsticales
            elif self.mode == 'A':
                ranges = list(self.scan.ranges)
                leftmean = np.mean(ranges[:30])
                rightmean = np.mean(ranges[690:])

                if abs(leftmean - rightmean) < 0.1:
                    #print "STUCK! Force turning!"
                    step.angular.z = 90 *2*pi/360
                    self.move_pub.publish(step)
                    time.sleep(1)
                else:
                    if leftmean > rightmean:
                        step.angular.z = -90 *2*pi/360
                    else:
                        step.angular.z = 90 *2*pi/360

                step.linear.x = 0.05

                #random turn: disabled
                if False:
                    rat = randint(1, 1000)
                    if rat == 1:
                        print "random avoid spining"
                        rat = randint(10,20)

                        timeout = time.time() + rat / 10.0
                        step.linear.x = 0
                        step.angular.z = 90 *2*pi/360

                        while True:
                            self.move_pub.publish(step)
                            if time.time() > timeout:
                                break
                            self.rate.sleep()

            #searching mode: approaching ball
            elif self.mode == 'B':
                step.linear.x = 0.3
                step.angular.z = self.ball_angle_offset * 2

            else:
                assert False, 'I don\'t recognize the current moving mode!  '+ str(self.mode)

            self.move_pub.publish(step)
            self.rate.sleep()

        step = Twist()
        self.move_pub.publish(step)
        self.found_ball = False

    #odom callback stores searching waypoints and keeps track of odom of crawbot
    def odom_callback(self, msg):
        self.crawbot_odom = msg
        (x, y) = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        #searching waypoints used to go back when a ball is found
        if len(self.waypoints) == 0:
            self.waypoints.append((x, y))
        else:
            (rx, ry) = self.waypoints[len(self.waypoints)-1]
            if sqrt( (rx-x)**2 + (ry-y)**2) > 0.5:
                self.waypoints.append((x, y))

    #scan callback stores lasercan informatoin
    def scan_callback(self, msg):
        self.scan = msg

        #set searching mode to avoid obsticales when too close to obsticales
        scan = list(self.scan.ranges[240:480])
        assert len(scan) != 0
        min_dis = numpy.nanmin(scan)

        if min_dis < 0.8:
            self.mode = 'A'
        elif self.mode != 'B':
            self.mode = 'S'

    #image callback keeps tracking of balls and moves to the next state when a ball is found
    def image_callback(self, msg):

        bot_odom = Odometry()
        bot_odom = self.crawbot_odom
        ranges = self.scan.ranges

        #convert raw camera info to opencv format
        bridge = CvBridge()
        try:
            image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        assert image.data != 0
        image = cv2.medianBlur(image,5)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #define color RED
        lower_red = np.array([0,100,100])
        upper_red = np.array([10,255,255])
        mask0 = cv2.inRange(hsv, lower_red, upper_red)
        lower_red = np.array([170,50,50])
        upper_red = np.array([180,255,255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        mask = mask0+mask1
        mask = cv2.blur(mask,(5,5))

        #detect balls in picture
        circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 2, 400,
                                    minRadius = 10, maxRadius = 500)
        #ball detected
        if circles is not None:
            if self.mode != 'A':
                self.mode = 'B'

            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv2.circle(image, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

                dis_pos = int(float(x)/float(800)*360)
                lc = list(ranges[180:540])

            (x,y,r) = circles[0]

            #calculate yaw offset between carbot and ball
            yaw_offset_deg = (float(400) - float(x)) / float(400) * float(45)
            yaw_offset = yaw_offset_deg * 0.0174533
            self.ball_angle_offset = yaw_offset

            balldis = np.mean(lc[dis_pos-5:dis_pos+5]) + 0.2

            #ball only counts when in the middle of camera and distance is less than 2 meters
            if 380 < x < 420 and  r < 100 and  balldis < 2:
                #print "Found balle ", balldis

                #calculate ball location on the map
                botx = self.crawbot_odom.pose.pose.position.x
                boty = self.crawbot_odom.pose.pose.position.y
                ori = self.crawbot_odom.pose.pose.orientation
                (roll,pitch,yaw) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
                yaw = yaw + yaw_offset
                ballx = botx + balldis * cos(yaw)
                bally = boty + balldis * sin(yaw)

                #print "bot(%f,%f) yaw(%f, %f) dis%f ball(%f,%f)" % (botx, boty, yaw,yaw_offset, r, ballx, bally)

                #ball is detected only within 1.3 meters
                if  balldis < 1.3:
                    #print "in range"
                    global nodename
                    homex = rospy.get_param("/"+nodename+"/xx")
                    homey = rospy.get_param("/"+nodename+"/yy")

                    if homex-2 < ballx < homex+2 and homey-2 < bally < homey+2:
                        #print "found ball at home! not picking up"
                        self.found_ball = False
                    else:
                        self.found_ball = True
                        #print "found ball ... going to pick it up"

                    global dis
                    dis = balldis

                #record ball locations: disabled
                if False:
                    already_found = False
                    for (xx,yy) in self.balls:
                        if abs(xx - ballx) < 0.4 and abs(yy - bally) < 0.4:
                            already_found = True
                            break

                    if not already_found:
                        self.balls.append((ballx, bally))
                        self.found_ball = True
                        print 'Found ball at (%f, %f)' %(ballx, bally)
                        print "Total # of ball is ", len(self.balls)
                            #print self.crawbot_odom

                if self.mode != 'A':
                    self.mode = 'S'

        #TODO: opencv windows freezes when moved to the next state
        #cv2.imshow("output", image)
        #cv2.waitKey(10)

#in the gettingball state, crawbot tries to grab the ball
#returns goinghome state
class GettingBall(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['none'])
        global nodename
        rospy.loginfo(nodename+":GettingBall initiation done.")


    def execute(self, userdata):
        global nodename
        rospy.loginfo(nodename+":Executing state GettingBall...")
        odom_sub = rospy.Subscriber(rn + "/odom", Odometry, self.odom_callback)
        self.lft_pub = rospy.Publisher(rn + '/left_finger_tip_controller/command', Float64, queue_size = 1)
        self.rft_pub = rospy.Publisher(rn + '/right_finger_tip_controller/command', Float64, queue_size = 1)
        self.move_pub = rospy.Publisher(rn + '/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.rate = rospy.Rate(60)

        self.getBall()
        return 'none'

    def getBall(self):
        time.sleep(1)
        self.turn_move_get()

    #main get ball method, turn 180 degrees, open gripper, move to ball, close gripper, and adjust
    def turn_move_get(self):
        #Turn 180 degree
        step = Twist()
        step.angular.z = 0.5
        ori=self.odom.pose.pose.orientation
        (roll,pitch,yaw_ori) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        yaw_ori = yaw_ori * 57.2958

        while True:
            self.move_pub.publish(step)
            now=self.odom.pose.pose.orientation
            (roll,pitch,yaw_now) = euler_from_quaternion([now.x, now.y, now.z, now.w])
            yaw_now = yaw_now * 57.2958

            yaw_diff = abs(yaw_ori-yaw_now)

            #print yaw_diff,yaw_ori,yaw_now

            if 177.5 < yaw_diff < 182.5:
                break
            self.rate.sleep()
        step.angular.z = 0
        self.move_pub.publish(step)

        #open gripper
        lft = Float64()
        rft = Float64()

        lft.data = 20
        rft.data = -20
        timeout = time.time() + 0.5
        while True:
            self.lft_pub.publish(lft)
            self.rft_pub.publish(rft)
            if time.time() > timeout:
                break
            self.rate.sleep()

        global dis
        #print dis

        # drive backward to ball
        timeout = time.time() + 5.5 + (dis-1)*5  # 5 seconds from now
        step.linear.x = -0.2
        step.angular.z = 0
        while True:
            self.move_pub.publish(step)
            if time.time() > timeout:
                break
            self.rate.sleep()
        step.linear.x = 0
        self.move_pub.publish(step)

        #grab ball
        lft.data = -20
        rft.data = 20
        timeout = time.time() + 2
        while True:
            self.lft_pub.publish(lft)
            self.rft_pub.publish(rft)
            if time.time() > timeout:
                break
            self.rate.sleep()

        self.rate.sleep()

        #adjust position to let ball fit in the gripper
        timeout = time.time() + .5
        step.linear.x = -0.2
        while True:
            self.move_pub.publish(step)
            if time.time() > timeout:
                break
            self.rate.sleep()
        step.linear.x = 0
        self.move_pub.publish(step)

        timeout = time.time() + .5
        step.linear.x = 0.1
        while True:
            self.move_pub.publish(step)
            if time.time() > timeout:
                break
            self.rate.sleep()
        step.linear.x = 0
        self.move_pub.publish(step)

    def odom_callback(self, msg):
        self.odom = msg

#in the goinghome state, crawbot goes home by following waypoitns created during searching state
#returns dropingball state
class GoingHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['none'])
        global nodename
        rospy.loginfo(nodename+":GoingHome initiation done.")

    def execute(self, userdata):
        global nodename
        rospy.loginfo(nodename+":Executing state GoingHome...")
        odom_sub = rospy.Subscriber(rn + "/odom", Odometry, self.odom_callback)
        self.move_pub = rospy.Publisher(rn + '/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        time.sleep(1)

        self.athome = False
        global waypoints
        self.waypoints = copy.deepcopy(waypoints)
        self.rate = rospy.Rate(60)

        self.gohome()
        return 'none'

    #main gohome method
    def gohome(self):
        self.waypoints.pop()
        self.waypoints.pop()
        wps = self.waypoints
        #print wps
        print "Total waypoints ", len(wps)

        #prune waypoints by rule #1
        #prune waypoints if they are in range of home: disabled
        if False:
            global nodename
            homex = rospy.get_param("/"+nodename+"/xx")
            homey = rospy.get_param("/"+nodename+"/yy")

            print homex, homey

            ii = 0
            for i in range(len(wps)-1, -1, -1):
                (x, y) = wps[i]
                if homex -1 < x < homex +1 and homey -1 < y < homey+ 1:
                    ii = i
                    #print "waypoints pruned: rule1"
                    break


            for j in range(0, ii):
                wps.pop(0)

            #print "pruned ", ii, " waypoints by rule #1"


        #prune waypoints by rule #2
        #prune waypoints that causes circles in the way back home
        wps_delete_mark = []
        for i in range(0, len(wps)):
            wps_delete_mark.append(False)

        #print wps
        #print wps_delete_mark

        for i in range(0, len(wps)):
            for j in range(0, i):
                (ix,iy) = wps[i]
                (jx,jy) = wps[j]
                if sqrt((ix-jx)**2+(iy-jy)**2) < 0.3:
                    print j,i
                    for k in range(j,i):
                        wps_delete_mark[k] = True

        #print wps
        #print wps_delete_mark

        pc = 0
        wpstmp = []
        for i in range(0,len(wps)):
            if not wps_delete_mark[i]:
                wpstmp.append(wps[i])
            else:
                pc+=1

        #print wpstmp
        wps = wpstmp
        #print wps
        #print "pruned ", pc, " waypoints by rule #2"

        #main gohome loop: pop a waypoint, turn crawbot to waypoint, and approach waypoint.
        while len(wps) > 0:

            (wpx, wpy) = wps.pop()

            #turn carwbot to waypoint
            botx = self.odom.pose.pose.position.x
            boty = self.odom.pose.pose.position.y

            #print botx,boty, wpx, wpy
            dx =  wpx -botx
            dy =  wpy - boty

            dis = sqrt( dx ** 2 + dy ** 2)

            wp_yaw = atan2(dy, dx)

            #print dx,dy,dis,wp_yaw

            #turn
            ori = self.odom.pose.pose.orientation
            (roll,pitch,yaw) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
            yawd = abs(wp_yaw - yaw)

            step = Twist()
            if wp_yaw > yaw:
                step.angular.z = 0.25
            else:
                step.angular.z = -0.25

            if wp_yaw * yaw < 1 and abs(wp_yaw) > 2 and abs(yaw) > 2:
                step.angular.z = -1 * step.angular.z

            while yawd > 0.015:
                self.move_pub.publish(step)

                ori = self.odom.pose.pose.orientation
                (roll,pitch,yaw) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

                yawd = abs(wp_yaw - yaw)
                #print yawd, wp_yaw, yaw

                self.rate.sleep()

            step.angular.z = 0
            self.move_pub.publish(step)



            botx = self.odom.pose.pose.position.x
            boty = self.odom.pose.pose.position.y

            #print wps
            #print botx,boty, wpx, wpy


            #move carwbot towards waypoint stop when within range 0.1 meters
            while dis > 0.1:
                step.linear.x = .6
                self.move_pub.publish(step)

                botx = self.odom.pose.pose.position.x
                boty = self.odom.pose.pose.position.y
                dx = botx - wpx
                dy = boty -wpy
                dis = sqrt( dx ** 2 + dy ** 2)
                #print dis

                self.rate.sleep()

            step.linear.x = 0
            self.move_pub.publish(step)


    def odom_callback(self, msg):
        self.odom = msg
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y


    def scan_callback(self, msg):
        self.scan = msg

#in state dropingball, carwbot drops the ball it is carrying
#returns resting state
class DropingBall(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['none'])
        global nodename
        rospy.loginfo(nodename+":DropingBall initiation done.")

    def execute(self, userdata):
        global nodename
        rospy.loginfo(nodename+":Executing state DropingBall...")

        odom_sub = rospy.Subscriber(rn + "/odom", Odometry, self.odom_callback)
        self.lft_pub = rospy.Publisher(rn + '/left_finger_tip_controller/command', Float64, queue_size = 1)
        self.rft_pub = rospy.Publisher(rn + '/right_finger_tip_controller/command', Float64, queue_size = 1)
        self.move_pub = rospy.Publisher(rn + '/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.rate = rospy.Rate(60)
        time.sleep(1)

        self.releaseball()
        return 'none'

    def releaseball(self):
        #Turn 180 degree to let the ball face home
        step = Twist()
        step.angular.z = 0.5
        ori=self.odom.pose.pose.orientation
        (roll,pitch,yaw_ori) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        yaw_ori = yaw_ori * 57.2958

        while True:
            self.move_pub.publish(step)
            now=self.odom.pose.pose.orientation
            (roll,pitch,yaw_now) = euler_from_quaternion([now.x, now.y, now.z, now.w])
            yaw_now = yaw_now * 57.2958

            yaw_diff = abs(yaw_ori-yaw_now)

            #print yaw_diff,yaw_ori,yaw_now

            if 175 < yaw_diff < 185:
                break
            self.rate.sleep()
        step.angular.z = 0
        self.move_pub.publish(step)

        #open gripper
        lft = Float64()
        rft = Float64()
        lft.data = 20
        rft.data = -20
        timeout = time.time() + .5
        while True:
            self.lft_pub.publish(lft)
            self.rft_pub.publish(rft)
            if time.time() > timeout:
                break
            self.rate.sleep()

        #push ball backward and adjust location of crawbot
        timeout = time.time() + 1
        step.linear.x = -0.5
        while True:
            self.move_pub.publish(step)
            if time.time() > timeout:
                break
            self.rate.sleep()
        step.linear.x = 0
        self.move_pub.publish(step)

        timeout = time.time() + 1
        step.linear.x = 0.5
        while True:
            self.move_pub.publish(step)
            if time.time() > timeout:
                break
            self.rate.sleep()
        step.linear.x = 0
        self.move_pub.publish(step)

        #close gripper
        lft.data = -20
        rft.data = 20
        timeout = time.time() + .5
        while True:
            self.lft_pub.publish(lft)
            self.rft_pub.publish(rft)
            if time.time() > timeout:
                break
            self.rate.sleep()


    def odom_callback(self, msg):
        self.odom = msg

#in the resting state, crawbot takes a nap for 3 seconds
#returns searching state
class Resting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['none'])
        global nodename
        rospy.loginfo(nodename+":Resting initiation done.")

    def execute(self, userdata):
        global nodename
        rospy.loginfo(nodename+":Executing state Resting...")
        rospy.loginfo(nodename+":I am taking a nap...")
        for i in range(0,3):
            time.sleep(1)
            rospy.loginfo(nodename+":...")
        return 'none'


#main method
def main(argv):

    #parse carwbot name from arguments
    rospy.init_node(sys.argv[1])

    global rn
    global nodename

    if sys.argv[1] == 'crawbot_smach1':
        rn = "/crawbot1"
    elif sys.argv[1] == 'crawbot_smach2':
        rn = "/crawbot2"
    else:
        assert False, "Wrong node name!"
    nodename = sys.argv[1]
    #print nodename, rn, "!!!!!!"

    #define the state machine
    sm = smach.StateMachine(outcomes=['success', 'fail'])
    #TODO: add data transitions betweens states to replace global variables
    with sm:
        smach.StateMachine.add('SEARCHING', Searching(),
            transitions={'foundBall':'GETTINGBALL'}
        )
        smach.StateMachine.add('GETTINGBALL', GettingBall(),
            transitions={'none':'GOINGHOME'}
        )
        smach.StateMachine.add('GOINGHOME', GoingHome(),
            transitions={'none':'DROPINGBALL'}
        )
        smach.StateMachine.add('DROPINGBALL', DropingBall(),
            transitions={'none':'RESTING'}
        )
        smach.StateMachine.add('RESTING', Resting(),
            transitions={'none':'SEARCHING'}
        )
    while not rospy.is_shutdown():
        outcome = sm.execute()

#entry of program
if __name__ == '__main__':
    main(sys.argv)

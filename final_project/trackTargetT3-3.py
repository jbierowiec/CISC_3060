# Program to track a colored target
# using gazebo camera in a window
# c dml 2018

import sys
import rospy
import cv2

from geometry_msgs.msg import Twist    
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# ROS topic names for turtlebot_gazebo
motionTopic ='/cmd_vel'
imageTopic = '/camera/rgb/image_raw'
headingTopic = '/heading'

# Global variables
gCurrentImage = Image() # make a global variable for the image
gBridge = CvBridge()    # make a ROS to CV bridge object

# Callback for the image topic
def callbackImage(img):
    '''Called automatically for each new image'''
    global gCurrentImage, gBridge
    gCurrentImage = gBridge.imgmsg_to_cv2(img, "bgr8")
    return

# procedure to track a colored region of the image by
# rotating the robot so that the colored region remains
# centered. The color information is a min and max 
# RGB value in targetCol=[minrgb,maxrgb]
def trackNode(targetCol):
    '''center the robot camera on the target if in view'''
    global gCurrentImage

    rospy.init_node('trackNode',anonymous=True)
    # create windows to show camera and processing
    cv2.namedWindow('Turtlebot Camera') # cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('Target')           # cv2.WINDOW_AUTOSIZE)

    imageSub = rospy.Subscriber(imageTopic,Image,callbackImage)
    heading_pub = rospy.Publisher(headingTopic, Twist, queue_size=10)
    rospy.sleep(5) # wait for callbacks to catch up

    rate = rospy.Rate(10)
    msg=Twist()
    start=rospy.get_time()

    while not rospy.is_shutdown():
        # just show what the camera sees now
        cv2.imshow('Turtlebot Camera', cv2.resize(gCurrentImage,(320,240)) )
        # get height h and width w of current image
        h,w = gCurrentImage.shape[0], gCurrentImage.shape[1]
        # make a binary image that is 0 except where the color is in range
        targetImage = cv2.inRange(gCurrentImage,targetCol[0],targetCol[1])
        cv2.imshow('Target',cv2.resize(targetImage,(320,240)) )
        
        # variable used to help stop loop and track how far is center
        trackCntr = 0

        # tracking algorithm
        avel=0.3 # default velocity, so robot 'spins' when no target in view
        lvel=0.0
        # extract the moments of the target image
        m = cv2.moments(targetImage)
        if m['m00']>0 and trackCntr > 50: # skip if the target image has non nonzero regions
            # how far is the X center of target  from X center of image
            delx = w/2 - m['m10']/m['m00'] 
            avel = 0.4*delx/w # use this to generate a proportional ang velocity
            dist= m['m00']/(h*w) # area as a fraction of the image size
            A,epi=70,10 # target area size, controls how close ti get to target
            if dist>A+epi:
                lvel = -0.1
            elif dist<A-epi:
                lvel=0.2
            else: # target now in range
                tnum = 1 # target number

                for i in tnum:
                    tnum = tnum + 1
                
                elapsed = rospy.get_time() - start
                txt1 = " Goal: " + str(round(tnum - 1, 2))
                txt2 = " Loc: " + str(round(dist, 2))
                txt3 = " Time to goal: " + str(round(elapsed, 2))
                font = cv2.FONT_SANS_SERIF
                black = (0, 0, 0)
                cv2.putText(gCurrentImage, txt1, (0, 100), font, 2, black)
                cv2.putText(gCurrentImage, txt2, (0, 150), font, 2, black)
                cv2.putText(gCurrentImage, txt3, (0, 200), font, 2, black)

                filename = "Goal"+str(tnum)+".jpg"

                cv2.imwrite(filename, gCurrentImage)
                return 

            print ("offset from image center =",round(delx,2),
                   " => avel=",round(avel,2))
            print ("size of target =",round(dist,2)," =>lvel=",round(lvel,2))
            trackCntr = round(dist,2)
        msg.linear.x,msg.angular.z=lvel,avel # publish velocity
        print(msg.linear.x, " ", msg.angular.z)
        heading_pub.publish(msg)
        cv2.waitKey(1) # need to do this for CV2 windows to show up
        rate.sleep()

    # path for image to be saved at 
    # imgPath = '~/CISC3060/3Dturtle/enclosedWorld/colMazePic'
    # saves the tutlebot camera image to the path
    # cv2.imwrite(gCurrentImage,imgPath)
    return

def callback_shutdown():
    print("Shutting down")
    pub = rospy.Publisher(motionTopic, Twist, queue_size=1)
    msg = Twist()
    msg.angular.z=0.0
    msg.linear.x=0.0
    pub.publish(msg) 
    rospy.sleep(5)
    return

if __name__ == '__main__':
    # identify/center the RGB color range of the target
    try:
        rospy.on_shutdown(callback_shutdown)
        
        # target colors
        targetColorO = [(0,30,75), (5,50,89)]         # orange target
        targetColorY = [(0,80,80), (10,110,110)]      # yellow target
        targetColorB = [(0,0,0), (20,20,20)]          # black target
        targetColorW = [(240,240,240), (255,255,255)] # white target 
        
        # array for colored targets
        targetColorArray = [targetColorO, targetColorY, targetColorB, targetColorW]
        
        # ffor loop to go through each target
        for i in targetColorArray: 
            trackNode(i)

    except  rospy.ROSInterruptException:
        pass

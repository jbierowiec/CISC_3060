# Bethany Fernandez & Jan C. Bierowiec
# Last Modified: 13 December 2022
# Obstacle Avoidance Node for the Final Project 

# import statements
import math
import numpy as np 
import matplotlib.pyplot as plt 
import random
import rospy 
import sys

from geometry_msgs.msg import Twist   # ROS Twist message 
from sensor_msgs.msg import LaserScan # ROS laser scan message 
from nav_msgs.msg import Odometry     # ROS odom message; helps with pos
from tf.transformations import euler_from_quaternion # Used to give rotation angle

# ROS Topics 
laserTopic = '/scan'      # gives us laser scan information 
motionTopic = '/cmd_vel'  # gives us angular and linear velocity
poseTopic = '/odom'       # gives us position

headingTopic = '/heading' # gives us the direction the robot should face
'''Heading Topic: Used as a means of communication between the color
   tracking node and the OA node.  
   The color tracking node finds the appropriate colored target.
   As the robot tries to center itself to the image of the target (move straight),
   it will send the direction to the OA node. '''

# Global variables 
gBumperLeft, gBumperRight= False,False # setting bumpers on the robot to not be activated.
gPose = [0,0,0]                        # x, y, z position 
gHeading = Twist()                     # creates a new Twist instance
gObstacleList = []
gLaserDist = 0.0

# Callback function provided by Dr. Damian Lyons.
def laserCallback(msg):
    '''Call back function for laser range data'''
    global gBumperLeft,gBumperRight, gObstacleList, gLaserDist
    gObstacleList = []
    
    gBumperLeft,gBumperRight=False,False
    numRays = len(msg.ranges) # total num readings
    radPerIndex = math.radians(360)/numRays
    cnt = 0 
    
    center = 0 # laser points to the front
    width = int(numRays/6) # left/right bumper 'window'
    tooClose=0.75 # threshold for bumper to activate
    
    for i in range(0,len(msg.ranges)):
        # rule out bad readings first
        if not math.isnan( msg.ranges[i] ) and \
           not math.isinf( msg.ranges[i] ) and msg.ranges[i]>0:
        
           # check for anything close left and right
           if msg.ranges[i]<tooClose:
               if i in range(0,width):
                   gBumperLeft=True
               elif i in range(numRays-width,numRays):
                   gBumpterRight=True
                   
           # translate every reading and map it
           angle = (i*radPerIndex) + gPose[2] 
           d = msg.ranges[i]
           glaserDist = d
           ox = gPose[0]+d*math.cos(angle)
           oy = gPose[1]+d*math.sin(angle)
           gObstacleList.append( (ox,oy) )
           cnt+=1

# Callback Function provided by Dr. Damian Lyons.
def poseCallback(data):
    global gPose
    gPose[0] = data.pose.pose.position.x
    gPose[1] = data.pose.pose.position.y
    orient = data.pose.pose.orientation
    quat = [axis for axis in [orient.x, orient.y, orient.z, orient.w]]
    (roll,pitch,yaw)=euler_from_quaternion(quat)
    gPose[2]=yaw  
    return
    
def headingCallback(msg):
    global gHeading 
    gHeading.linear.x = msg.linear 
    gHeading.angular.z= msg.angular

# Euclidean distance
def eucdist(x1,y1,x2,y2):
    d=0.0
    delx = x2-x1
    dely = y2-y1
    dsqr = delx*delx+dely*dely
    if dsqr>0:
        d = math.sqrt(dsqr)
    return d

# attractive field; function provided by Dr. Damian Lyons 
def attract_pot(rx, ry, gx, gy):
    '''returns component of vector field due to attraction'''
    K = 0.5 # attractive gain
    delx = gx - rx
    dely = gy - ry
    angle = math.atan2(dely,delx)
    mag = eucdist(rx, ry, gx, gy)
    return ( K*mag*math.cos(angle), K*mag*math.sin(angle) )

# repulsive field; function provided by Dr. Damian Lyons 
def repulse_pot(rx, ry, ox, oy):
    '''returns component of vector field due to repulsion'''
    Dmax=1.5 # repulsive boundary
    K = 0.05 # repulsive gain
    delx = rx - ox
    dely = ry - oy
    angle = math.atan2(dely,delx)
    d = eucdist(rx, ry, ox, oy)
    if d>Dmax:
        return (0,0)
    mag = ( (1.0/d) - (1.0/Dmax) ) * (1.0/d)**2
    return (K*mag*math.cos(angle),K*mag*math.sin(angle) )

# summation of fields/forces; function provided by Dr. Damian Lyons 
def pot_field(rx,ry,gx,gy, olist):
    '''sums attractive and repulsive potential fields'''
    
    attr_field = np.array( attract_pot(rx, ry, gx, gy) )
    
    repl_field=(0.0,0.0)
    for (ox,oy) in olist:
        repl_field = repl_field + np.array( repulse_pot(rx, ry, ox, oy))
    if eucdist(repl_field[0],repl_field[1],0,0)>0.001:
        print (gPose[0:2],repl_field)     
    
    field = attr_field + repl_field

    norm = eucdist(field[0],field[1],0.0,0.0)
    field = field/norm

    return field

def oa_node():
    rospy.init_node('OA_Node', anonymous = True) # initialize the node 
    
    # Set up Publishers
    pub = rospy.Publisher(motionTopic, Twist, queue_size = 10)

    # Set up Subscribers
    rospy.Subscriber(laserTopic, LaserScan, laserCallback)
    rospy.Subscriber(poseTopic, Odometry, poseCallback)    
    rospy.Subscriber(headingTopic, Twist, headingCallback) # provided by CT node

    rate = rospy.Rate(10)   # rate is 10 Hz
    
    # Declare global variables in function 
    global gHeading, gPose, gLaserDist, gObstacleList, gLaserDist
    
    prior_avel,prior_lvel= gHeading.angular.z, gHeading.linear.x
    
    # Obstacle Avoidance Algorithm 
    while not rospy.is_shutdown():
        
        msg = Twist()
        
        if gBumperLeft or gBumperRight:
            print('Bump!')
        
        if prior_lvel != 0:     
            target = pot_field(gPose[0],gPose[1],gPose[0]+ gLaserDist ,gPose[1] + gLaserDist, gObstacleList)
            targetTheta = math.atan2(target[1],target[0])
            delTheta =  targetTheta - gPose[2]
            mag = eucdist(target[0],target[1],0,0)
            
            # check for error 'wrapping around'
            if delTheta>math.pi or delTheta<-math.pi:
                if targetTheta>0:
                    delTheta -= 2*math.pi
                else:
                    delTheta += 2*math.pi
        
            # scale velocities
            avel = 1.0 *(delTheta)
            lvel = 0.3 * mag
 
            # smooth the velocity signal
            msg.angular.z = 0.5*(avel+prior_avel)
            msg.linear.x = 0.5*(lvel+prior_lvel)
            pub.publish(msg)
            
            prior_avel,prior_lvel=avel,lvel
        
        msg.linear.x, msg.angular.z = prior_lvel, prior_avel
        pub.publish(msg) 
            
        rate.sleep()
            
        # print("Done ","d=",round(eucdist(goalx,goaly,gPose[0], gPose[1]),2),end="")
        # print(" Loc= ",round(gPose[0],2), round(gPose[1],2))
    return

# Shutdown function; important for any node involving movement 
def callback_shutdown():
    print("Shutting down...")
    pub = rospy.Publisher(motionTopic, Twist, queue_size = 10)
    msg = Twist() #creates ROS message that determines lin. and ang. velocities.
    msg.linear.x, msg.angular.z = 0,0 #send message to stop the motors. 
    pub(msg)
    
    return
    
#---------------------------------MAIN------------------------------------
if __name__ == "__main__":
    try: 
        #Insert Main Stuff here
        oa_node()
    except rospy.ROSInterruptException:
        pass

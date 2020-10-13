#!/usr/bin/env python
#mehmet calikus   150150042
import rospy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
import tf
from tf import transformations
import math

waypoint=None

#waypoint callback
def waypoint_callback(msg): #  callback

    #***************************************
    #***          Obtain current destination
    #***************************************

    #save waypoint data for printing out in main loop
    global waypoint
    waypoint=msg;


if __name__ == '__main__':

    #setup ROS node, subscribe waypoint_cb to the topic /waypoint_cmd & publish motor commands
    rospy.init_node("crazy_driver_456")
    waypoint_subscriber = rospy.Subscriber("/waypoint_cmd", Transform, waypoint_callback) # <--- set up callback
    motor_command_publisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=100)
    #you could in principle also subscribe to the laser scan as is done in assignment 1.

    #setup transform cache manager
    listener = tf.TransformListener()

    #start a loop; one loop per second
    delay = rospy.Rate(1.0); # perhaps this could be faster for a controller?
    q_prev = 0
    while not rospy.is_shutdown():


        #***************************************
        #***          Obtain current robot pose
        #***************************************
        
        try:
            #grab the latest available transform from the odometry frame (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.
            
            (translation,orientation) = listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0));
        except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("EXCEPTION:",e)
            #if something goes wrong with this just go to bed for a second or so and wake up hopefully refreshed.
            delay.sleep()
            continue
        

        #***************************************
        #***          Print current robot pose
        #***************************************

        #Print out the x,y coordinates of the transform
        print("Robot is believed to be at (x,y): (",translation[0],",",translation[1],")")

        #Convert the quaternion-based orientation of the latest message to Euler representation in order to get z axis rotation
        r_xorient, r_yorient, r_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(orientation))
        robot_theta = r_zorient  # only need the z axis
        print("Robot is believed to have orientation (theta): (",robot_theta,")\n")

        #***************************************
        #***          Print current destination
        #***************************************

        # the waypoint variable is filled in in the waypoint_callback function above, which comes from incoming messages
        # subscribed to in the .Subscriber call above.

        #Print out the x,y coordinates of the latest message
        print("Current waypoint (x,y): (",waypoint.translation.x,",",waypoint.translation.y,")")

        #Convert the quaternion-based orientation of the latest message to angle-axis in order to get the z rotation & print it.
        waypointrotq = [waypoint.rotation.x,waypoint.rotation.y,waypoint.rotation.z,waypoint.rotation.w]
        w_xorient, w_yorient, w_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(waypointrotq))
        waypoint_theta=w_zorient # only need the z axis
        print("Current waypoint (theta): (",waypoint_theta,")\n")

        #***************************************
        #***          DRIVE THE ROBOT HERE (same as with assignment 1 
        #**           - except you are driving towards a goal not away from an obstacle)
        #***************************************

        #for containing the motor commands to send to the robot
        motor_command=Twist()

        #The following formulation used to convert real-world's coordinates to coordinates in the robot's world
        
        # difference of x-axis between current state and desired(destination) state
        distance_x = waypoint.translation.x - translation[0]  
        
        # difference of y-axis between current state and desired(destination) state
        distance_y = waypoint.translation.y - translation[1]  

        #variables that used for rotate the robot to the desired state
        xr = math.cos(robot_theta) * distance_x + math.sin(robot_theta) * distance_y
        yr = math.cos(robot_theta) * distance_y - math.sin(robot_theta) * distance_x


        p = math.sqrt((xr * xr) + (yr * yr)) # distance between waypoint and robot -> distance error
##############################################################################################################################        
# atan2 method cannot be used for dor on route 4, since the return value is in the range of -pi and +pi,  
#since the speed is already multiplied by minus 1, atan2 method may obstruct the robot going back because of cosine value of some angles
# In order to get rid of this situation, the atan method that return a value between -pi/2 and pi/2 is used 
#cosine value of angles between pi/2 an -pi/2 cannot be minus, the problem that the robot cannot go back has been eliminated. 

        q1 = math.atan(yr / xr) # rotational defference between waypoint's pointing direction and robot's pointing direction (4 dor)
        q2 = math.atan2(yr, xr) # rotational defference between waypoint's pointing direction and robot's pointing direction (1,2,3,4 dis; 1,2,3, dor)
#############################################################################################################################
  
        #Used P control on linear velocity
        Kp = 0.8  # proportionality constant(gain)
        #Used PD style control on angular velocity
        Kp_2 = 0.90 #proportional gain 
        Kd = 0.50 #derivative gain

###########################################################################################################################
#       These conditions are set for 4th dor section, if the angle between waypoint and robot smaller than pi/2 degrees
#       also location of x-axis of waypoint less than robot's x-axes robot can understand that 
#       waypoint is behind of it and moves back with minus speed, for this purpose its speed multiply with -1.
        ang_dif_right = (robot_theta - waypoint_theta) % (2 * math.pi) 
        ang_dif_left = (waypoint_theta - robot_theta) % (2 * math.pi) 

        #route 4 dor
        if (xr < 0) and ((ang_dif_right < (math.pi / 2)) or (ang_dif_left < (math.pi /2))):
            motor_command.linear.x= -1 * Kp * p * math.cos(abs(q1))                         
            motor_command.angular.z=Kp_2*q1 + Kd*(q1-q_prev) #q_prev defined as zero at line 37
    
            q_prev = q1
       

        #the other all stituations where waypoints(desired points) in front of the robot 
        else:   
            motor_command.linear.x=Kp*p*math.cos(abs(q2))  #Cosine is used for reducing speed before turning for smoothness
            motor_command.angular.z=Kp_2*q2 + Kd*(q2-q_prev) #q_prev defined as zero at line 37
    
            q_prev = q2


############################################################################################################################        



    
        motor_command_publisher.publish(motor_command)
        
    
        delay.sleep()
        # we don't need to call spinOnce like in roscpp as callbacks happen in different threads
    
    
    print("ROS shutdown now I will also go to sleep. I hope I didn't crash. Night night.")

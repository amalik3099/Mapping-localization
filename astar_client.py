#!/usr/bin/env python
import rospy
import math

from nav_msgs.srv import GetPlan, GetMap

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class AstarClient:

    def __init__(self):
        """
        Class constructor
        """
        # initialize current pose variables
        self.px = 0
        self.py = 0
        self.pth = 0
        self.gtx = 0
        self.gty = 0
        self.gtth = 0
        self.orient = Quaternion()

        rospy.init_node('astar_client', anonymous=True)
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        self.astar_publish = rospy.Publisher('/publish_point', PoseStamped, queue_size = 100)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.astar_client)

        #service used by master to decide where to move to next
        movement_service = rospy.Service('movement_service', GetPlan, self.automatic_astar_client)

        self.rate = rospy.Rate(10)

    def astar_client(self, msg):
        #astar_client implements the service when a message is received from /move_base_simple/goal
        rospy.loginfo("waiting for plan_path service")
        create_plan = GetPlan()
        start = PoseStamped()
        #the starting location is the robot's current pose
        start.pose.position.x = self.px
        start.pose.position.y = self.py
        start.pose.position.z = 0
        start.pose.orientation = self.orient
        start.header.stamp = rospy.Time.now()

        # goal is the pose given in the message
        goal = msg

        create_plan.start = start
        create_plan.goal = goal
        create_plan.tolerance = 0.01
        rospy.wait_for_service('/plan_path')

        rospy.loginfo('plan_path service exists')
        astar_path = rospy.ServiceProxy('/plan_path', GetPlan)
        #service is used with start, goal and tolerance
        resp1 = astar_path(start, goal, 0.01)
        rospy.loginfo('plan_path received')
        path = resp1.plan
        self.drive_path(path)

    def automatic_astar_client(self, msg):
        #astar_client implements the service when a message is received from /move_base_simple/goal
        rospy.loginfo("waiting for plan_path service")
        create_plan = GetPlan()
        start = PoseStamped()
        #the starting location is the robot's current pose
        start.pose.position.x = self.px
        start.pose.position.y = self.py
        start.pose.position.z = 0
        start.pose.orientation = self.orient
        start.header.stamp = rospy.Time.now()

        #end goal is the pose given in the message depending if it is a GetPlan type or PoseStamped
        goal = msg.goal

        create_plan.start = start
        create_plan.goal = goal
        create_plan.tolerance = 0.01
        rospy.wait_for_service('/plan_partial')

        rospy.loginfo('plan_partial service exists')
        astar_path = rospy.ServiceProxy('/plan_partial', GetPlan)
        #service is used with start, goal and tolerance
        resp1 = astar_path(start, goal, 0.01)
        rospy.loginfo('plan_partial received')
        path = resp1.plan
        self.drive_path(path)

        return path

    def drive_path(self, path):
        for p in path.poses:
            rospy.loginfo(p)
            self.go_to(p)

    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### Makes a new Twist message
        msg_cmd_vel = Twist()
    	# Linear velocity
    	msg_cmd_vel.linear.x = linear_speed
    	msg_cmd_vel.linear.y = 0.0
    	msg_cmd_vel.linear.z = 0.0
    	# Angular velocity
    	msg_cmd_vel.angular.x = 0.0
    	msg_cmd_vel.angular.y = 0.0
    	msg_cmd_vel.angular.z = angular_speed
        ### Publishes the message
        self.cmd_vel.publish(msg_cmd_vel)


    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        current_pose = [self.px, self.py, self.pth]
    	initial_pose = current_pose
        # final pose is distance to be moved by the robot in the x direction
    	distance_traveled = 0
    	tolerance = 0.01

        self.send_speed(linear_speed, 0.0)
    	while abs(distance-distance_traveled) > tolerance:
            current_pose = [self.px, self.py, self.pth]
            distance_traveled = math.sqrt((current_pose[0]-initial_pose[0])*(current_pose[0]-initial_pose[0])+(current_pose[1]-initial_pose[1])*(current_pose[1]-initial_pose[1]))
            #print(final_pose[0]-current_pose[0])
    	self.send_speed(0.0,0.0)


    def rotate(self, angle, aspeed):
        """
        Rotates the robot arounabs(d the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        current_pose = [self.px, self.py, self.pth]
        initial_pose = current_pose
        # final pose is the final angle that the robot moves to about z
        final_angle = self.pth+angle
        if final_angle < self.pth:
            aspeed=aspeed*(-1)

        final_pose = [self.px, self.py, final_angle]
    	tolerance = 0.01

        self.send_speed(0.0, aspeed)
        while abs(final_pose[2]-current_pose[2]) > tolerance:
            current_pose = [self.px, self.py, self.pth]
        self.send_speed(0.0, 0.0)

    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        rospy.loginfo('starting go_to')
        #assuming msg is the click
        #rotate to face base of arrow
        quat_orig = msg.pose.orientation
        quat_list = [ quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        #calculates roll, pitch, and yaw from quaternion
        (roll , pitch , yaw) = euler_from_quaternion( quat_list )

        self.gtx = msg.pose.position.x
        trans_x = self.gtx - self.px
        self.gty = msg.pose.position.y
        trans_y = self.gty - self.py

        rotation = math.atan2(trans_y, trans_x)
        self.rotate(rotation-self.pth, 0.5)

        #go-to translation position
        distance = math.sqrt((trans_x)*(trans_x) + (trans_y)*(trans_y))

        self.drive(distance, 0.2)

        #rotate to orientation of arrow
        self.gtth = yaw -self.pth
        #self.rotate(self.gtth, 0.5)

    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        self.px = msg.pose.pose.position.x
    	self.py = msg.pose.pose.position.y
    	quat_orig = msg.pose.pose.orientation
    	quat_list = [ quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
    	#calculates roll, pitch, and yaw from quaternion
        self.orient = msg.pose.pose.orientation
        (roll , pitch , yaw) = euler_from_quaternion( quat_list )
    	self.pth = yaw


    def arc_to(self, position):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code



    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    AstarClient().run()
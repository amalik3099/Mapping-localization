#!/usr/bin/env python

import math
import rospy
import cv2
from rbe3002_lab4.srv import frontierservice
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
import priority_queue
from path_planner import PathPlanner


class Master:

    def __init__(self):
        """
        Class constructor
        """
        #initializes variables
        self.orient = Quaternion()
        self.mapreceived = False
        self.mapdata = OccupancyGrid()

        #Launces node and subscribes to gmapping map topic
        rospy.init_node('Master', anonymous=True)
        rospy.Subscriber('/map', OccupancyGrid, self.request_map)

        self.rate = rospy.Rate(10)
        rospy.sleep(1.0)

    def request_map(self, map):
        """
        Requests the map from the gmapping
        :return [OccupancyGrid] The C-Space.
        """
        rospy.loginfo("Requesting the map")
        #sets the self variables so that they can be accessed in other functions
        self.mapdata = map
        self.mapreceived = True


    def send_goal(self, posx, posy):
        """
        Uses the frontier coordinates provided and uses /movement_service to
        make a service call to direct the robot to generate a path from
        current location to goal
        """
        rospy.loginfo("waiting for movement_service")
        plan = GetPlan()

        goal = PoseStamped()
        #the goal location is the robot's desired pose
        goal.pose.position.x = posx
        goal.pose.position.y = posy
        goal.pose.position.z = 0.0
        goal.pose.orientation = self.orient
        goal.header.stamp = rospy.Time.now()

        #start is initialized to zero because only goal is used in service
        start = PoseStamped()
        start.pose.position.x = 0.0
        start.pose.position.y = 0.0
        start.pose.position.z = 0.0
        start.pose.orientation = self.orient
        start.header.stamp = rospy.Time.now()

        #plan incorporates start, goal, and tolerance
        plan.goal = goal
        plan.start = start
        plan.tolerance = 0.01
        rospy.wait_for_service('/movement_service')

        rospy.loginfo('movement_service exists')
        sent_goal = rospy.ServiceProxy('/movement_service', GetPlan)

        response = sent_goal(start, goal, plan.tolerance)
        rospy.loginfo('movement plan received')


    def choose_frontier(self, map):
        """
        Provides the map from the gmapping to frontier service
        :return [Path] a list of poses of frontier centroid coordinates.
        """
        rospy.loginfo("waiting for frontier service")
        rospy.wait_for_service('/frontier_finder')
        rospy.loginfo('frontier service exists')
        sent_map = rospy.ServiceProxy('/frontier_finder', frontierservice)

        response = sent_map(map)
        rospy.loginfo('map received')

        return response


    def run(self):
        """
        phase 1 checks for frontiers and navigates to them
        until no more frontiers are available.
        """

        while 1:
            #checks if a map has been received
            if self.mapreceived:
                #uses service call to retrieve list of frontier centroids
                frontiers_path = self.choose_frontier(self.mapdata)
                #attempts to send robot to centroid of largest frontier
                try:
                    PathPlanner.map = self.mapdata
                    self.send_goal(frontiers_path.goal.poses[0].pose.position.x, frontiers_path.goal.poses[0].pose.position.y)
                #if it fails, loop through other frontiers and attempt to navigate to those
                except:
                    for i in frontiers_path.goal.poses:
                    PathPlanner.map = self.mapdata
                        self.send_goal(i.pose.position.x, i.pose.position.y)
                    self.mapreceived = False
                self.mapreceived = False


if __name__ == '__main__':
    Master().run()
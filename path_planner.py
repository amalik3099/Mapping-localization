#!/usr/bin/env python

import math
import rospy
import cv2
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
import priority_queue

cost_so_far = {}
xy_frontier_cells = []

class PathPlanner:

    def __init__(self):
        """
        Class constructor
        """
        self.orig_map = OccupancyGrid()
        self.map = OccupancyGrid()
        ## Initializes the node and call it "path_planner"
        rospy.init_node("path_planner")

        rospy.Subscriber('/map', OccupancyGrid, self.request_map)

        ## Creates a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        plan_service = rospy.Service('plan_path', GetPlan, self.plan_path)

        #seperate service for planning partial paths vs full paths
        partial_plan_service = rospy.Service('plan_partial', GetPlan, self.plan_partial_path)

        ## Creates a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.cspace = rospy.Publisher('/path_planner/cspace', GridCells, queue_size=10)
        ## Creates publishers for A*, expanded cells, and frontier
        self.astar = rospy.Publisher('/path_planner/astar', GridCells, queue_size=10)
        self.expanded_cells = rospy.Publisher('/path_planner/expanded_cells', GridCells, queue_size=10)
        self.frontier = rospy.Publisher('/path_planner/frontier', GridCells, queue_size=10)

        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")

    @staticmethod
    def round_down(n, decimals=0):
        #allows floats to be rounded down to ints
        multiplier = 10 ** decimals
        return math.floor(n * multiplier) / multiplier

    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        width = mapdata.info.width
        index = int(y * width + x)
        return index

    @staticmethod
    def index_to_grid(mapdata, index):
        """
        Returns the x y coordinates corresponding to the given index
        :param index [int] The cell index.
        :return  (int, int) The x y coordinate of the cell in the grid.
        """
        point = ()
        width = mapdata.info.width
        height = mapdata.info.height
        pos_x = (index%width)

        height_multiplier = PathPlanner.round_down(index/height)
        pos_y = height_multiplier

        point = (pos_x,pos_y)
        return point


    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """
        hofn = math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
        return hofn


    @staticmethod
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        # wc_x and wc_y are the world coordinates
        # resolution is the map resolution
        # origin.position.x and origin.position.y are the position of the origin in the world
        wc_x = (x + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.x
        wc_y = (y + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.y
        wc = (wc_x,wc_y)
        return wc


    @staticmethod
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        # gc_x and gc-y are the grid coordinates
        # resolution is the map resolution
        gc_x = int((wp.x - mapdata.info.origin.position.x) / mapdata.info.resolution)
        gc_y = int((wp.y - mapdata.info.origin.position.y) / mapdata.info.resolution)
        gp = (gc_x,gc_y)
        return gp



    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        pose_stamped_list = []
        path.append(path[-1])
        for p in range(0,len(path)-1):
            curr_cell = path[p]
            next_cell = path[p+1]

            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'map' #world
            pose.pose.position.x = curr_cell.x #p_world[0]
            pose.pose.position.y = curr_cell.y #p_world[1]
            pose.pose.position.z = 0

            pose.pose.orientation.w = math.atan2(next_cell.y-curr_cell.y, next_cell.x-curr_cell.x)

            pose_stamped_list.append(pose)
        return pose_stamped_list



    @staticmethod
    def is_cell_walkable(mapdata, x, y):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [int]           Returns int corresponding to cell value
        """
        #checks if cell is occupied by an obstacle or not based on the occupancy grid
        if (0 <= x < mapdata.info.width) and (0 <= y < mapdata.info.height):
            index_check = PathPlanner.grid_to_index(mapdata, x, y)
            if (mapdata.data[index_check] == 100):
                return 100
            elif mapdata.data[index_check] == -1:
                return -1
            else:
                return 1 # means cell is walkable
        else:
            return -2 #means out of bounds


    @staticmethod
    def neighbors_of_4(mapdata, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        north = (x, y + 1)
        south = (x, y - 1)
        east = (x + 1, y)
        west = (x - 1, y)

        walkable_4_neighbors = []
        # for neighbors of four, positions north, south, east, west are checked to see if walkable
        if PathPlanner.is_cell_walkable(mapdata, north[0], north[1]) == 1:
            walkable_4_neighbors.append(north)
        if PathPlanner.is_cell_walkable(mapdata, south[0], south[1]) == 1:
            walkable_4_neighbors.append(south)
        if PathPlanner.is_cell_walkable(mapdata, east[0], east[1]) == 1:
            walkable_4_neighbors.append(east)
        if PathPlanner.is_cell_walkable(mapdata, west[0], west[1]) == 1:
            walkable_4_neighbors.append(west)

        return walkable_4_neighbors

    @staticmethod
    def neighbors_of_8(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        north_east = (x + 1, y + 1)
        north_west = (x - 1, y + 1)
        south_east = (x + 1, y - 1)
        south_west = (x - 1, y - 1)

        walkable_8_neighbors = []
        # for neighbors of eight, positions north,north east,northwest, south,southeast, south west, east, and west are checked to see if walkable
        if PathPlanner.is_cell_walkable(mapdata, north_east[0], north_east[1]) == 1:
            walkable_8_neighbors.append(north_east)
        if PathPlanner.is_cell_walkable(mapdata, south_east[0], south_east[1]) == 1:
            walkable_8_neighbors.append(south_east)
        if PathPlanner.is_cell_walkable(mapdata, north_west[0], north_west[1]) == 1:
            walkable_8_neighbors.append(north_west)
        if PathPlanner.is_cell_walkable(mapdata, south_west[0], south_west[1]) == 1:
            walkable_8_neighbors.append(south_west)

        walkable_4 = PathPlanner.neighbors_of_4(mapdata, x, y)
        walkable_8_neighbors.extend(walkable_4)
        return walkable_8_neighbors

    @staticmethod
    def unknown_neighbors_of_4(mapdata, x, y):
        """
        Returns the unknown 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of unknown 4-neighbors.
        """
        north = (x, y + 1)
        south = (x, y - 1)
        east = (x + 1, y)
        west = (x - 1, y)

        unknown_4_neighbors = []
        # for neighbors of four, positions north, south, east, west are checked to see if walkable
        if PathPlanner.is_cell_walkable(mapdata, north[0], north[1]) == -1:
            unknown_4_neighbors.append(north)
        if PathPlanner.is_cell_walkable(mapdata, south[0], south[1]) == -1:
            unknown_4_neighbors.append(south)
        if PathPlanner.is_cell_walkable(mapdata, east[0], east[1]) == -1:
            unknown_4_neighbors.append(east)
        if PathPlanner.is_cell_walkable(mapdata, west[0], west[1]) == -1:
            unknown_4_neighbors.append(west)

        return unknown_4_neighbors

    @staticmethod
    def unknown_neighbors_of_8(mapdata, x, y):
        """
        Returns the unknown 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of unknown 8-neighbors.
        """
        north_east = (x + 1, y + 1)
        north_west = (x - 1, y + 1)
        south_east = (x + 1, y - 1)
        south_west = (x - 1, y - 1)

        unknown_8_neighbors = []
        # for neighbors of eight, positions north,north east,northwest, south,southeast, south west, east, and west are checked to see if walkable
        if PathPlanner.is_cell_walkable(mapdata, north_east[0], north_east[1]) == -1:
            unknown_8_neighbors.append(north_east)
        if PathPlanner.is_cell_walkable(mapdata, south_east[0], south_east[1]) == -1:
            unknown_8_neighbors.append(south_east)
        if PathPlanner.is_cell_walkable(mapdata, north_west[0], north_west[1]) == -1:
            unknown_8_neighbors.append(north_west)
        if PathPlanner.is_cell_walkable(mapdata, south_west[0], south_west[1]) == -1:
            unknown_8_neighbors.append(south_west)

        unknown_4 = PathPlanner.unknown_neighbors_of_4(mapdata, x, y)
        unknown_8_neighbors.extend(unknown_4)
        return unknown_8_neighbors



    def request_map(self, map):
        """
        Requests the map from the gmapping
        :return [OccupancyGrid] The C-Space.
        """
        rospy.loginfo("Requesting the map")
        self.map = map
        self.orig_map = self.map


    def calc_cspace(self, mapdata, padding=3):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [[int8]]        The C-Space as a list of values from 0 to 100.
        """
        rospy.loginfo("Calculating C-Space")

        ## Goes through each cell in the occupancy grid
        ## Inflates the obstacles where necessary based on padding argument
        self.orig_map = self.map
        new_map = []
        print "calc_cspace: ", type(mapdata)
        total_range = mapdata.info.width * mapdata.info.height
        for p in range(0,padding):
            copy = list(mapdata.data)
            for i in range(0, total_range):
                if mapdata.data[i] == 100:
                    #coordinates are found via indexing through occupancy grid.
                    #Index to grid is used to convert the index to useable grid coordinates
                    coordinates = PathPlanner.index_to_grid(mapdata, i)
                    pos_x = coordinates[0]
                    pos_y = coordinates[1]

                    neighbors = PathPlanner.neighbors_of_8(mapdata, pos_x, pos_y)

                    #iterates through neighbors list and changes all available surrounding neighbors to obstacles
                    for n in neighbors:
                        index_value = PathPlanner.grid_to_index(mapdata, n[0], n[1])
                        copy[index_value] = 100
                        world_pos = PathPlanner.grid_to_world(mapdata, n[0], n[1])
                        cell_point = Point()
                        cell_point.x = world_pos[0]
                        cell_point.y = world_pos[1]
                        cell_point.z = 0
                        new_map.append(cell_point)

            mapdata.data = copy

        ## Creates a GridCells meassage and publish it
        grid_cells_message = GridCells()
        grid_cells_message.header.frame_id = 'map'
        grid_cells_message.cell_width = mapdata.info.resolution
        grid_cells_message.cell_height = mapdata.info.resolution
        grid_cells_message.cells = new_map

        ## Returns the C-space
        self.cspace.publish(grid_cells_message)

        return mapdata.data


    def a_star(self, mapdata, start, goal):
        """
        Calculates the astar path.
        Publishes the list of cells that were added to the path.
        :param mapdata [OccupancyGrid] The map data.
        :param start [int8,int8]     The x,y grid location of the starting location
        :param goal [int8,int8]     The x,y grid location of the goal location
        :return        [[int8]]        The C-Space as a list of path values.
        """
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        frontier_cells = []
        unoptimized_path = []
        visited_cells = []
        #initializes dictionaries and lists
        frontier = priority_queue.PriorityQueue()
        frontier.put(start,0)
        came_from = {}
        came_from[start]= None
        cost_so_far = {}
        cost_so_far[start] = 0

        #only when not empty
        while not frontier.empty():
            current = frontier.get()
            #have we reached?
            if current == goal:
                break
            #current cell is turned into a Point class and added to the visited_cells list
            visited_cell = Point()
            world_pos_astar = PathPlanner.grid_to_world(mapdata, current[0], current[1])
            visited_cell.x = world_pos_astar[0]
            visited_cell.y = world_pos_astar[1]
            visited_cell.z = 0
            visited_cells.append(visited_cell)

            #finds neighbor list in order to start A* algorithm
            list_of_8_neighbors = PathPlanner.neighbors_of_8(mapdata, current[0], current[1])
            #iterates through all 8 neighbors to determine cost at each node (f(n))
            for next in list_of_8_neighbors:
                next_x = next[0]
                next_y = next[1]
                #h(n) cost from neighbor. Heuristic is euclidiean distance
                new_cost = cost_so_far[current] + PathPlanner.euclidean_distance(next_x, next_y, current[0], current[1])
                #checks to see how the cost of the next neighbor compares to the visited_cell
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    #updates cost_so_far[] dictionary to include new neighbor
                    cost_so_far[next]=new_cost
                    priority = new_cost + PathPlanner.euclidean_distance(next_x, next_y, goal[0], goal[1])
                    frontier.put(next,priority)
                    #generates list of frontier_cell Points to use in rviz
                    frontier_cell = Point()
                    world_pos_astar = PathPlanner.grid_to_world(mapdata, next[0], next[1])
                    frontier_cell.x = world_pos_astar[0]
                    frontier_cell.y = world_pos_astar[1]
                    frontier_cell.z = 0
                    frontier_cells.append(frontier_cell)
                    came_from[next] = current

                    #publishes frontier in rviz
                    frontier_cells_message = GridCells()
                    frontier_cells_message.header.frame_id = 'map'
                    frontier_cells_message.cell_width = mapdata.info.resolution
                    frontier_cells_message.cell_height = mapdata.info.resolution
                    frontier_cells_message.cells = frontier_cells
                    ## Return the C-space
                    self.frontier.publish(frontier_cells_message)

                    #generates list of visited cells and publishes to rviz
                    visited_cells_message = GridCells()
                    visited_cells_message.header.frame_id = 'map'
                    visited_cells_message.cell_width = mapdata.info.resolution
                    visited_cells_message.cell_height = mapdata.info.resolution
                    visited_cells_message.cells = visited_cells
                    ## Return the C-space
                    self.expanded_cells.publish(visited_cells_message)
        node = goal

        #Iterates through came_from[] dictionary backwards to find A* path
        while node != start:
            node = came_from[node]

            unoptimized_path_cell = Point()
            world_pos_unoptimized_path = PathPlanner.grid_to_world(mapdata, node[0], node[1])
            unoptimized_path_cell.x = world_pos_unoptimized_path[0]
            unoptimized_path_cell.y = world_pos_unoptimized_path[1]
            unoptimized_path_cell.z = 0
            unoptimized_path.append(unoptimized_path_cell)

        # message is published and displayed in rviz
        unoptimized_path_message = GridCells()
        unoptimized_path_message.header.frame_id = 'map'
        unoptimized_path_message.cell_width = mapdata.info.resolution
        unoptimized_path_message.cell_height = mapdata.info.resolution
        unoptimized_path_message.cells = unoptimized_path
        ## Return the C-space

        return unoptimized_path

    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        n = 0
        slope_first_second = 0
        slope_second_third = 1
        intercept_first_second = 2
        intercept_second_third = 3
        while path[n] != path[-2]:
            first = path[n]
            second = path[n+1]
            third = path[n+2]
            #only if change in x is not 0, calculate slopes and intercepts
            if (second.x - first.x) != 0 and (third.x - second.x) != 0:
                slope_first_second = (second.y - first.y)/(second.x - first.x)
                slope_second_third = (third.y - second.y)/(third.x - second.x)
                intercept_first_second = int((1)-slope_first_second*(2))
                intercept_second_third = int((1)-slope_second_third*(2))

            #checks if points are parrallel using slope, and checks if points are colinear using intercepts
            if (slope_first_second == slope_second_third and intercept_first_second == intercept_second_third) or (first.x == second.x == third.x) or (first.y == second.y == third.y):
                #print("12: %s" %intercept_first_second)
                path.remove(second)
                n = 0
            else:
                n = n + 1
        path.reverse()
        del path[0]
        return path

    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path_to_poses(mapdata, path):path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        rospy.loginfo("Returning a Path message")
        display_path_message = GridCells()
        display_path_message.header.frame_id = 'map'
        display_path_message.cell_width = mapdata.info.resolution
        display_path_message.cell_height = mapdata.info.resolution
        display_path_message.cells = path
        self.astar.publish(display_path_message)

        optimized_path_message = PathPlanner.path_to_poses(mapdata,path)
        path_final = Path()
        path_final.header.frame_id = 'map'
        path_final.poses = optimized_path_message

        return path_final



    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req
        """
        ## Requests the map
        ## In case of error, return an empty path
        mapdata = self.map

        if mapdata is None:
            return Path()
        ## Calculate the C-space and publish it
        print "plan_path: ", type(mapdata)
        mapdata.data = self.calc_cspace(mapdata, 2)

        ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)

        path  = self.a_star(mapdata, start, goal) #cspacedata

        ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)

        ## Return a Path message
        return self.path_to_message(mapdata, waypoints)

    def plan_partial_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req
        """
        ## Requests the map
        ## In case of error, return an empty path
        if self.map != self.orig_map:
            mapdata = self.orig_map

        else:
            mapdata = self.map

        if mapdata is None:
            return Path()
        ## Calculate the C-space and publish it
        print "plan_path: ", type(mapdata)
        mapdata.data = self.calc_cspace(mapdata, 2)

        ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        #print type(start)
        path  = self.a_star(mapdata, start, goal)

        ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)

        halfwaypoint = len(waypoints)/2
        # deletes waypoints in order to reduce path size to half size so that
        #robot can check gmapping more frequently and adjust path
        while len(waypoints) > 2:
            del waypoints[-1]

        # Return a Path message
        return self.path_to_message(mapdata, waypoints)



    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()



if __name__ == '__main__':
    PathPlanner().run()
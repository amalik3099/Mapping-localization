#!/usr/bin/env python

import math
import rospy
import cv2
from nav_msgs.srv import GetPlan, GetMap
from rbe3002_lab4.srv import frontierservice
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
import priority_queue
from path_planner import PathPlanner
from operator import itemgetter

class Frontier:

    def __init__(self):
        """
        Class constructor
        """

        ## Initializes the node and call it "frontier"
        rospy.init_node("Frontier")

        frontier_service = rospy.Service('frontier_finder', frontierservice, self.find_frontier)

        rospy.sleep(1.0)
        rospy.loginfo("frontier node ready")

    def calc_cspace(self, mapdata, padding):
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
        new_map = []
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

        return mapdata.data

    def find_frontier(self, msg):
        """
        Takes in map [OccupancyGrid] from service call
        return: [Path] list of PoseStamped frontier centroids
        """
        #cspace is calculated to find walkable area
        msg.map.data = self.calc_cspace(msg.map, 2)

        #variables are initialized
        walkable_cells = []
        cells_with_unknown_neighbors = []
        centroids = []
        distances = []
        frontiers = []
        frontier_locations = []
        sorted_distances = []
        total_range = msg.map.info.width * msg.map.info.height

        mapdata = msg.map

        #finds walkable cells in cspace
        for i in range(0, total_range):
            coordinate = PathPlanner.index_to_grid(mapdata, i)
            pos_x = coordinate[0]
            pos_y = coordinate[1]
            if PathPlanner.is_cell_walkable(mapdata, pos_x, pos_y) == 1:
                walkable_cells.append(coordinate)

        # finds all cells that are neighboring an unknown cell(value = -1)
        for c in walkable_cells:
            neighbors = PathPlanner.unknown_neighbors_of_8(mapdata, c[0], c[1])

            for n in neighbors:
                if c not in cells_with_unknown_neighbors:
                    cells_with_unknown_neighbors.append(c)

        # finds which cells in cells_with_unknown_neighbors neighbor each other
        # and generates a list of those that touch
        frontiers = Frontier.separate(cells_with_unknown_neighbors)

        for f in frontiers:
            if len(f) < 2:
                frontiers.remove(f)

        for f in frontiers:
            #finds centroids of all contiguous frontier groups
            centroid_value = Frontier.calc_centroid(f)
            final = centroid_value

            #finds the walkable cell that is closest to the centroid location
            for k in walkable_cells:
                neighbors = PathPlanner.neighbors_of_8(mapdata, k[0], k[1])
                if len(neighbors) == 8:
                    distToCentroid = PathPlanner.euclidean_distance(k[0], k[1], final[0], final[1])
                    distances.append((distToCentroid, k))
            #list of distances is sorted in descending order in terms of size
            sorted_distances = sorted(distances)

        for s in sorted_distances:
            final = s[1]
            world_coord = PathPlanner.grid_to_world(mapdata, final[0], final[1])

            frontier_location = PoseStamped()
            frontier_location.pose.position.x = world_coord[0]
            frontier_location.pose.position.y = world_coord[1]
            frontier_location.pose.position.z = 0.0
            frontier_location.pose.orientation = Quaternion()
            frontier_location.header.stamp = rospy.Time.now()
            frontier_locations.append(frontier_location)

        all_frontiers = Path()
        all_frontiers.header.frame_id = 'map'
        all_frontiers.poses = frontier_locations

        return all_frontiers


    @staticmethod
    def separate(data):
        """
        takes in a list of cell coordinates and groups them by adjacency
        """
        record = dict()
        sorted_data = sorted(data)
        for x, y in sorted_data:
            if x in record.keys():
                record[x][y] = None
            else:
                record[x] = dict()
                record[x][y] = None

            def in_record(x, y):
                """ returns true if x, y is in record """
                return x in record.keys() and y in record[x].keys()

            def find_adjacents(x, y):
                """ find adjacent cells (8 directions) """
                check = [((x - 1, y - 1), in_record(x - 1, y - 1)),
                         ((x + 1, y + 1), in_record(x + 1, y + 1)),
                         ((x - 1, y + 1), in_record(x - 1, y + 1)),
                         ((x + 1, y - 1), in_record(x + 1, y - 1)),
                         ((x - 1, y), in_record(x - 1, y)),
                         ((x + 1, y), in_record(x + 1, y)),
                         ((x, y - 1), in_record(x, y - 1)),
                         ((x, y - 1), in_record(x, y - 1))]

                result = []
                for coord, found in check:
                    if found:
                        result.append(coord)
                return result

            def get_group(cells):
                """ Returns group id number of first of cells, or None """
                for x, y in cells:
                    if record[x][y] is not None:
                        return record[x][y]
                return None

        # loop over each, assigning a group id
        group_id = 0
        for x, y in sorted_data:
            cells = find_adjacents(x, y)
            if len(cells) == 0:
                # no neighbors found, make a new group
                record[x][y] = group_id
                group_id += 1
            else:
                # found cells
                found_group = get_group(cells)
                if found_group is None:
                    # no neighbors have a group
                    # give this cell and all neighbors a new group
                    record[x][y] = group_id
                    for f_x, f_y in cells:
                        record[f_x][f_y] = group_id
                    group_id += 1
                else:
                    # found a neighbor in a group
                    # apply that group to this and all other neighbors
                    record[x][y] = found_group
                    for f_x, f_y in cells:
                        record[f_x][f_y] = found_group

        result = []
        for i in range(0, group_id):
            result.append([])
            for x, y_dict in record.items():
                for y, group in y_dict.items():
                    if group == i:
                        result[i].append( (x, y) )
        result = [r for r in result if len(r) != 0]
        return result

    @staticmethod
    def calc_centroid(cells):
        """
        takes list of cells in a frontier
        return: [tuple] centroid by averaging x and y
        """
        x_total = 0
        y_total = 0
        num_elements = len(cells)
        for cell in cells:
            x_total += cell[0]
            y_total += cell[1]
        centroid_x = x_total/num_elements
        centroid_y = y_total/num_elements
        centroid = (centroid_x, centroid_y)

        return centroid

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


if __name__ == '__main__':
    Frontier().run()
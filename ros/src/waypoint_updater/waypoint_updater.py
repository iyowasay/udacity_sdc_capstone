#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
from scipy.spatial import KDTree

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb) 
        rospy.Subscriber('/obstacle_waypoint', , self.obstacle_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None 
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1
        
        self.loop()
        # rospy.spin()

    def loop(self):
        rate = rospy.Rate(50) # 50 Hz, update rate? minimum 30 Hz
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                self.publish_waypoints()
            rate.sleep() 

    def get_closest_waypoint_idx(self):
        x = self.pos.pos.position.x
        y = self.pos.pos.position.y
        closest_index = self.waypoint_tree.query([x,y], 1)[1]

        # check if the closest point is ahead or behind the vehicle using basic math
        closest_coord = self.waypoints_2d[closest_index]
        prev_coord = self.waypoints_2d[closest_index-1]
        # equation for hyperplane through closest_coords
        cl_vec = np.array(closest_coord)
        prev_vec = np.array(prev_coord)
        pos_vec = np.array([x,y])
        # check if the dot product of two vector is positive or negative
        val = np.dot(cl_vec-prev_vec, pos_vec-cl_vec)
        if val > 0:
            closest_index = (closest_index+1) % len(self.waypoints_2d)
        return closest_index

    def publish_waypoints(self):
        final_lane = generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()
        # lane.header = self.base_waypoints.header
        closest_index = get_closest_waypoint_idx()
        farthest_index = closest_waypoint_idx + LOOKAHEAD_WPS
        lane_wps = self.base_waypoints.waypoints[closest_index:farthest_index]

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_index):
            lane.waypoints = lane_wps
        else:
            lane.waypoints = self.generate_decelerate_waypoints(lane_wps, closest_index)
        
        return lane

    def generate_decelerate_waypoints(self, waypoints, closest_idx):
        new_waypoints = []

        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0) 
            # - 2: two waypoints backward since we want the front edge to stop at the stop line, not the center line of the vehicle.
            dist = self.distance(waypoints, i, stop_idx)
            # distance will return 0 if i > stop_idx ???

            vel = math.sqrt(2*MAX_DECEL*dist)
            if vel < 1.:
                vel = 0.

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            new_waypoints.append(p)
        return new_waypoints


    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        # make sure that the 2d waypoints is initialized before subscriber is called.
        if not self.waypoints_2d: 
            self.waypoints_2d = [[waypoint.pose.pose.position.x waypoint.pose.pose.position.y] for waypoint in waypoints.waypoint]
            self.waypoint_tree = KDTree(self.waypoints_2d)


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        self.

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

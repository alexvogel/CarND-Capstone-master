#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

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

        self.current_pos_sub = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.len_final_waypoints = 200
        self.len_waypoints = 9999
        self.pose = None
        self.waypoints = None

        #main loop
        self.loop()

        rospy.spin()

    def loop(self):
        # the main publisher loop
        rate = rospy.Rate(50)
        while (self.pose is None) or (self.waypoints is None):
            rate.sleep()
        
        rospy.loginfo("in waypoint_updater: got pos and weypoints") 
        while not rospy.is_shutdown():
            rate.sleep()
            final_waypoints = self.get_final_waypoints()
            self.final_waypoints_pub.publish(final_waypoints)
                
    def get_final_waypoints(self):
        new_waypoints = []
        curr_waypoint_i = self.get_curr_waypoint_index()
        for i in range(self.len_final_waypoints):
            i_next_waypoint = (curr_waypoint_i+i)%self.len_waypoints
            new_waypoints.append(self.waypoints[i_next_waypoint])
        lane = Lane()
        lane.waypoints=new_waypoints
        lane.header.stamp = rospy.Time.now()
        return lane

    def get_curr_waypoint_index(self):
        min_i= 0
        min_d= 9999999;
        pos_x = self.pose.position.x
        pos_y = self.pose.position.y
        for i, w in enumerate(self.waypoints):
            w_x = w.pose.pose.position.x
            w_y = w.pose.pose.position.y
            distance = math.sqrt( ((pos_x-w_x)**2) + ((pos_y-w_y)**2)  )
            if distance < min_d:
                min_d = distance
                min_i = i
        return min_i

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg.pose

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.waypoints = waypoints.waypoints
        self.len_waypoints = len(self.waypoints)
        self.base_waypoints_sub.unregister()

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

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

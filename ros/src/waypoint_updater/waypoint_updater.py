#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import tf

import copy

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

# temporaty, will use a polynom for best deceleration curve later
DECEL_NB = 50
# TBD: get the good value
TARGET_SPEED = 11.1111 # m/s, about 25 mph

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        
        # I haven't seen anything about obstacle yet
        #rospy.Subscriber('/obstacle_waypoint', None, self.obstacle_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Internal members
        
        # Data about the world
        self.waypoints = None
        self.nb_wp = 0
        
        # last known car position/speed
        self.pose = None
        self.last_idx = -1
        self.velocity = None
        
        # info about trafic lights
        self.stop_wp = -1
        self.speed_profile = None
        
        # parameters
        self.nb_ahead = rospy.get_param('~waypoint_lookahead_nb', LOOKAHEAD_WPS)
        
        # start the main loop
        self.loop()
        
    '''
    This is the main loop of the node.
    On each iteration (based on waypoint_updater_freq parameter) the node will publish
    a list of waypoints ahead of the car.
    '''
    def loop(self):
        freq = rospy.get_param('~waypoint_updater_freq', 10)
        rate = rospy.Rate(freq)
        while not rospy.Time.now().to_sec():
            pass
        while not rospy.is_shutdown():
            if self.waypoints is None or self.pose is None:
                pass
            else:
                if self.last_idx < 0:
                    self.initial_index_update()
                self.index_update()
                
            rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg.pose

    def waypoints_cb(self, waypoints):
        if self.waypoints is None:
            self.waypoints = waypoints.waypoints
            self.nb_wp = len(self.waypoints)
            rospy.loginfo('Waypoints received. There are %s waypoints', self.nb_wp)
        else:
            rospy.logerr('We have got more than one base waypoint definition')

    def traffic_cb(self, msg):
        if msg.data+1 < self.last_idx:
            # Stop waypoint is -1 or behind us, no stop
            if self.stop_wp > 0:
                rospy.loginfo("Restart")
            self.stop_wp = -1
            self.speed_profile = None
            return
        if self.stop_wp != msg.data:
            rospy.loginfo("We will stop at WP %s", msg.data)
            self.stop_wp = msg.data
            self.compute_deceleration_profile()
        else:
            # we will keep the previous profile
            pass
    
    def velocity_cb(self, msg):
        self.velocity = msg.twist.linear.x
        

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
    
    def compute_deceleration_profile(self):
        if self.last_idx == -1 or self.velocity is None:
            rospy.loginfo("Cannot compute deceleration profile if I don't know the speed or the car position")
            self.stop_wp = -1
            return
        
        # naive implementation, linear compute_deceleration
        # TODO improve this
        self.speed_profile = copy.deepcopy(self.waypoints[self.stop_wp-DECEL_NB+1:self.stop_wp+1])
        for i in range(DECEL_NB):
            self.speed_profile[-i].twist.twist.linear.x *= i/DECEL_NB
            self.speed_profile[-i].twist.twist.angular.z *= i/DECEL_NB
        
    
    def initial_index_update(self):
        rospy.logdebug("Initial index update")
        # find the closest WP (will manage the car direction afterwards, using the index_update function, called in the callback)
        min_dist = float("inf")
        min_idx = -1
        
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        
        #rospy.logerr("Initial called %s", self.pose)
        for idx, wp in enumerate(self.waypoints):
            d = dl(self.pose.position, wp.pose.pose.position)
            if d <= min_dist:
                min_dist = d
                self.last_idx = idx
        
        if self.last_idx == -1:
            rospy.logerr("Something has gone horribly wrong here, probably an empty WP initialisation?")
        else:
            rospy.loginfo("First waypoint index at initialisation: %s", self.last_idx)
            
    def index_update(self):
        # check if the wp is in front of car. If not, takes the next one
        
        # we're a car, 2D geometry should be good enough
        _,_,yaw = tf.transformations.euler_from_quaternion(
            [self.pose.orientation.x,
             self.pose.orientation.y,
             self.pose.orientation.z,
             self.pose.orientation.w])
        
        while self.last_idx < self.nb_wp:
            dx = self.waypoints[self.last_idx].pose.pose.position.x-self.pose.position.x
            dy = self.waypoints[self.last_idx].pose.pose.position.y-self.pose.position.y
            
            dx_in_car = math.cos(yaw)*dx + math.sin(yaw)*dy
            if dx_in_car > 0:
                break
            self.last_idx += 1
        rospy.logdebug("Current WP index: %s", self.last_idx)
        
        lane = Lane()
        
        if self.stop_wp <= self.last_idx:
            lane.waypoints = self.waypoints[self.last_idx:self.last_idx+self.nb_ahead]
        else:
            lane.waypoints = copy.deepcopy(self.waypoints[self.last_idx:self.last_idx+self.nb_ahead])
            
            # whatever, just to avoid overshoot, temporary:
            j = self.nb_ahead
            
            for i in range(DECEL_NB):
                if i >= DECEL_NB or self.speed_profile is None:
                    break
                lane.waypoints[i+self.stop_wp-self.last_idx-DECEL_NB] = self.speed_profile[i]
                
                j = i+self.stop_wp-self.last_idx-DECEL_NB
                
            # temporaty
            for i in range(j, self.nb_ahead):
                lane.waypoints[i].twist.twist.linear.x = 0
                
        
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time.now()
        
        self.final_waypoints_pub.publish(lane)
        


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

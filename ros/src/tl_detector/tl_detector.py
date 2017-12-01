#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 2
LOOKAHEAD_WPS = 200 # Number of waypoints ahead our vehicle where to search for traffic lights

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector') #, log_level=rospy.WARN)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.last_wp = None

        self.visibility = rospy.get_param('~waypoint_lookahead_nb', LOOKAHEAD_WPS)

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        #keep trace of the last known position of our vehicle
        self.last_pos = -1

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''

        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            rospy.logdebug('publishing WP: %s', light_wp)
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            rospy.logdebug('publishing wp: %s', self.last_wp)
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        #extract car yaw data from quaternion
        _,_,car_yaw = tf.transformations.euler_from_quaternion(
            [pose.orientation.x,
             pose.orientation.y,
             pose.orientation.z,
             pose.orientation.w]
        )

        #rospy.logdebug('position: (%f,%f)', pose.position.x, pose.position.y)

        #loop through waypoints starting from few waypoints before our last known position (to limit computational time)
        #wp_idx = max(0, self.last_pos - 300)
        #if self.last_pos == len(self.waypoints.waypoints) - 1:
        #    wp_idx = 0
        i = 0
        wp_idx = 0
        dist = None
        min_dist = None

        #while (dist == None or dist < 0) and i < len(self.waypoints.waypoints):
        while i < len(self.waypoints.waypoints):
            wp_x = self.waypoints.waypoints[i].pose.pose.position.x
            wp_y = self.waypoints.waypoints[i].pose.pose.position.y

            dx = wp_x - pose.position.x
            dy = wp_y - pose.position.y

            #calculate the distance in car coordinates between the car and the waypoint
            car_dx = math.cos(-car_yaw) * dx - math.sin(-car_yaw) * dy
            car_dy = math.sin(-car_yaw) * dx + math.cos(-car_yaw) * dy
            dist = math.sqrt(car_dx**2 + car_dy**2)

            #rospy.logdebug('wp %i (%f,%f): dist: %f (dx:%f, dy:%f)', i, wp_x, wp_y, dist, dx, dy)
            if car_dx > 0 and (min_dist == None or dist < min_dist):
                min_dist = dist
                wp_idx = i
                #rospy.logdebug('dist_dx: (%f,%f), yaw: %f, idx: %i, dist_sqrt: %f', car_dx, car_dy, car_yaw, wp_idx, dist)
            i += 1

        rospy.logdebug('nearest waypoint: (%f,%f) - idx: %i, dist: %s', wp_x, wp_y, wp_idx, min_dist)

        return wp_idx

    def get_closest_trafficlight(self, pose):
        """Identifies the closest path waypoint to the given position

        Args:
            pose (Pose): position to match a traffic light to

        Returns:
            (float, float): position of the closest traffic light.

        """

        #extract car yaw data from quaternion
        _,_,car_yaw = tf.transformations.euler_from_quaternion(
            [pose.orientation.x,
             pose.orientation.y,
             pose.orientation.z,
             pose.orientation.w]
        )

        min_dist = None
        nearest_tl = None
        tl_idx = -1
        i = 0

        for light in self.lights:
            tl_x = light.pose.pose.position.x
            tl_y = light.pose.pose.position.y

            dx = tl_x - pose.position.x
            dy = tl_y - pose.position.y

            #calculate the distance in car coordinates between the car and the traffic light
            car_dx = math.cos(-car_yaw) * dx - math.sin(-car_yaw) * dy
            car_dy = math.sin(-car_yaw) * dx + math.cos(-car_yaw) * dy
            dist = math.sqrt(car_dx**2 + car_dy**2)

            #rospy.logdebug('light nr. %i: (%f,%f) - dist: %f', i, tl_x, tl_y, dist)
            if (car_dx > 0 and (min_dist == None or min_dist > dist)):
                min_dist = dist
                tl_idx = i
                nearest_tl = (tl_x, tl_y, tl_idx)

            i += 1

        if (nearest_tl != None):
            rospy.logdebug('nearest light: (%f,%f) - idx: %i, dist: %s', nearest_tl[0], nearest_tl[1], nearest_tl[2], min_dist)
        else:
            rospy.logdebug('nearest light: NONE')

        #if (tl_idx > -1 and nearest_tl != None):
        #    rospy.logdebug('nearest semaphore: (%f,%f) - idx: %i', nearest_tl[0], nearest_tl[1], nearest_tl[2])

        #return nearest traffic light index
        return tl_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        #TODO: temporarily I'm returning the color state included in the light data, this will have to be replaced by the classifier
        # rospy.logdebug('light state: %s', light.state)
        # return light.state

        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        #find the closest traffic light to the vehicle (if one exists)
        car_idx = -1
        tl_idx = -1
        if(self.pose):
            rospy.logdebug('finding car waypoint:')
            car_idx = self.get_closest_waypoint(self.pose.pose)
            self.last_pos = car_idx
            rospy.logdebug('finding traffic light index:')
            tl_idx = self.get_closest_trafficlight(self.pose.pose)

        #rospy.loginfo('vehicle position: (%f,%f) - wp index: %i', self.pose.pose.position.x, self.pose.pose.position.y, car_idx)

        #if a traffic light has been found, find its nearest waypoint
        tl_wp_idx = -1
        if tl_idx > -1:
            tl_pos = self.lights[tl_idx].pose.pose
            rospy.logdebug('finding traffic light waypoint:')
            tl_wp_idx = self.get_closest_waypoint(tl_pos)
            light = self.lights[tl_idx]

        #if the waypoint for this traffic light is too far from us, ignore it
        if (tl_wp_idx == -1):
            rospy.logdebug('no traffic light has been found ahead')
        elif (tl_wp_idx - car_idx > self.visibility):
            tl_wp_idx = -1
            rospy.logdebug('traffic light too far, will be ignored')
        else:
            rospy.logdebug('traffic light index: %i', tl_wp_idx)

        stop_line_wp = None
        #find the waypoint corresponding to the stop line for the closest traffic light (if one exists)
        rospy.logdebug('car wp index: %i', car_idx)
        rospy.logdebug('tl wp index: %i (%i)', tl_wp_idx, tl_idx)
        if (tl_idx > -1 and tl_wp_idx > -1):
            stop_line_pos = stop_line_positions[tl_idx]
            #I remove "3" from the stop line X because the value is refered to the center of the vehicle, so we need to stop a bit earlier than that
            stop_line_x = stop_line_pos[0]
            stop_line_y = stop_line_pos[1]
            min_dist = None
            for idx in range(max(0, tl_wp_idx-100), tl_wp_idx):
                tmp_wp = self.waypoints.waypoints[idx]
                dx = stop_line_x - tmp_wp.pose.pose.position.x
                dy = stop_line_y - tmp_wp.pose.pose.position.y
                dist = math.sqrt(dx**2 + dy**2)
                if min_dist == None or dist < min_dist:
                    min_dist = dist
                    stop_line_wp = idx

        if stop_line_wp != None:
            rospy.logdebug('stop line index: %i (%f,%f)', stop_line_wp, self.waypoints.waypoints[stop_line_wp].pose.pose.position.x, self.waypoints.waypoints[stop_line_wp].pose.pose.position.y)

        '''
        if car_wp > -1:
            rospy.logdebug('car position: %i (%f,%f)', car_wp, self.waypoints.waypoints[car_wp].pose.pose.position.x, self.waypoints.waypoints[car_wp].pose.pose.position.y)
        else:
            rospy.logdebug('car position: not found')
        if tl_wp_idx > -1:
            rospy.logdebug('light position: %i (%f,%f)', tl_wp_idx, self.waypoints.waypoints[tl_wp_idx].pose.pose.position.x, self.waypoints.waypoints[tl_wp_idx].pose.pose.position.y)
        else:
            rospy.logdebug('light position: not found')
        if car_wp > -1 and tl_wp_idx > -1:
            if tl_wp_idx > car_wp:
                rospy.logdebug('distance: %i', tl_wp_idx - car_wp)
            else:
                rospy.logdebug('something went wrong, traffic light waypoint is behind us')
        '''

        if light:
            state = self.get_light_state(light)
            return stop_line_wp, state
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

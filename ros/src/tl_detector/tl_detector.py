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

STATE_COUNT_THRESHOLD = 3
LOOKAHEAD_WPS = 500 # Number of waypoints ahead our vehicle where to search for traffic lights

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.INFO)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.last_wp = None

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

        self.visbility = rospy.get_param('~waypoint_lookahead_nb', LOOKAHEAD_WPS)

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

        rospy.loginfo('%i waypoints received', len(self.waypoints.waypoints))

    def traffic_cb(self, msg):
        self.lights = msg.lights

        #rospy.loginfo('%i traffic lights received', len(self.lights))

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
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
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

        #rospy.loginfo('car pose: (%f,%f)', pose.position.x, pose.position.y)

        if self.last_wp == None:
            self.last_wp = 0
        wp_idx = self.last_wp
        dist = None
        while dist == None or dist < 0:
            wp_x = self.waypoints.waypoints[wp_idx].pose.pose.position.x
            wp_y = self.waypoints.waypoints[wp_idx].pose.pose.position.y

            dx = wp_x - pose.position.x
            dy = wp_y - pose.position.y

            #calculate the distance in car coordinates between the car and the waypoint
            car_dx = math.cos(car_yaw) * dx - math.sin(car_yaw) * dy
            dist = car_dx

            if (car_dx > 0):
                break
            wp_idx += 1

        #rospy.loginfo('nearest waypoint: (%f,%f) - idx: %i', wp_x, wp_y, wp_idx)

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

        rospy.loginfo('car pose: (%f,%f)', pose.position.x, pose.position.y)

        dist = None
        nearest_tl = None
        tl_idx = -1
        i = 0
        for light in self.lights:
            tl_x = light.pose.pose.position.x
            tl_y = light.pose.pose.position.y

            dx = tl_x - pose.position.x
            dy = tl_y - pose.position.y

            #calculate the distance in car coordinates between the car and the traffic light
            car_dx = math.cos(car_yaw) * dx - math.sin(car_yaw) * dy
            rospy.loginfo('light nr. %i: (%f,%f) - dist: %f', i, tl_x, tl_y, car_dx)
            if (car_dx > 0 and (dist == None or dist > car_dx)):
                dist = car_dx
                tl_idx = i
                nearest_tl = (tl_x, tl_y, tl_idx)

            i += 1

        rospy.loginfo('nearest semaphore: (%f,%f) - idx: %i', nearest_tl[0], nearest_tl[1], nearest_tl[2])

        '''
        if nearest_tl > -1:
            #position that correspond to the line to stop in front of for the current traffic light (if found)
            stop_line_position = self.config['stop_line_positions']
        '''




        #return nearest_tl
        return tl_idx

    def get_stopline_from_waypoint(self, wp_idx, stop_line_pos):
        """Finds the stop line corresponding to the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        return 0

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        #extract car yaw data from quaternion
        _,_,car_yaw = tf.transformations.euler_from_quaternion(
            [self.pose.pose.orientation.x,
             self.pose.pose.orientation.y,
             self.pose.pose.orientation.z,
             self.pose.pose.orientation.w]
        )

        #rospy.loginfo('%i semaphores received', len(self.lights))
        #(per debug, potrei mostrare l'elenco di tutti i waypoints e di tutti i semafori - piu fattibile i semafori, i waypoints sono troppi mi sa)
    	#rospy.loginfo('x:%s, y:%s, z:%s',msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        #rospy.loginfo('current car position: (%f, %f) heading: %f', self.pose.pose.position.x, self.pose.pose.position.y, car_yaw)

        #this is a tuple that will contain the data of the closest semaphore (x,y,yaw)
        nearest_tl = None;
        #this variable will contain the closest semaphore index
        nearest_tl_idx = -1
        min_dist = -1

        i = 0
        for light in self.lights:
            i += 1

            #extract semaphore yaw data from quaternion
            _,_,sem_yaw = tf.transformations.euler_from_quaternion(
                [light.pose.pose.orientation.x,
                 light.pose.pose.orientation.y,
                 light.pose.pose.orientation.z,
                 light.pose.pose.orientation.w]
            )

            #TODO: check if there is a better way to identify similar orientation (given that the range is -3/0/+3, so a value of -3 is similar to +3)
            #check if the semaphore orientation is similar to our car orientation (+-0.4)
            '''orient_delta = 0.4
            if car_yaw >= sem_yaw - orient_delta or car_yaw <= sem_yaw + orient_delta or (abs(car_yaw) >= 2.6 and abs(sem_yaw) >= 2.6 and (abs(car_yaw) - abs(sem_yaw) <= orient_delta)):
                dist = math.sqrt((self.pose.pose.position.x - light.pose.pose.position.x)**2 + (self.pose.pose.position.y - light.pose.pose.position.y)**2)
                if dist < min_dist or min_dist < 0:
                    min_dist = dist
                    nearest_tl = (light.pose.pose.position.x, light.pose.pose.position.y)
            '''

            #rospy.loginfo('light %i: (%f, %f), heading: %f', i, light.pose.pose.position.x, light.pose.pose.position.y, sem_yaw)
        #rospy.loginfo('__________________________')

        '''
        if nearest_tl != None:
            rospy.loginfo('nearest traffic light location: (%f,%f)', nearest_tl[0], nearest_tl[1])
        '''

        #filter only waypoints ahead of our vehicle

        #find nearest waypoint
        #(the semaphore needs to have same heading (+-0.4) of the car)
        #(the semaphore needs to have same x (-3) and same y (+-4?) of the stop line coordinates)



        #convert to local coordinates
        local_x = self.pose.pose.position.x * math.cos(car_yaw) - self.pose.pose.position.y * math.sin(car_yaw)
        local_y = self.pose.pose.position.x * math.sin(car_yaw) + self.pose.pose.position.y * math.cos(car_yaw)
        #rospy.loginfo('car local coords (%f,%f)', local_x, local_y)

        #rospy.loginfo('__________________________')



        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        #get the closest waypoint and the closest traffic light to the vehicle
        if(self.pose):
            car_wp = self.get_closest_waypoint(self.pose.pose)
            tl_idx = self.get_closest_trafficlight(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)
        if tl_idx > -1:
            tl_pos = self.lights[tl_idx].pose.pose
            tl_pos.position.x = stop_line_positions[tl_idx][0]
            tl_pos.position.y = stop_line_positions[tl_idx][1]
            tl_wp = self.get_closest_waypoint(tl_pos)

        '''
        #find the waypoint corresponding to the stop line for the closest traffic light (if one exists)
        if (tl_idx > -1 and tl_wp > -1):
            stop_line_wp = get_stopline_from_waypoint(tl_wp, stop_line_positions[tl_idx])
        '''

        if car_wp > -1:
            rospy.loginfo('car position: %i (%f,%f)', car_wp, self.waypoints.waypoints[car_wp].pose.pose.position.x, self.waypoints.waypoints[car_wp].pose.pose.position.y)
        else:
            rospy.loginfo('car position: not found')
        if tl_wp > -1:
            rospy.loginfo('light position: %i (%f,%f)', tl_wp, self.waypoints.waypoints[tl_wp].pose.pose.position.x, self.waypoints.waypoints[tl_wp].pose.pose.position.y)
        else:
            rospy.loginfo('light position: not found')
        if car_wp > -1 and tl_wp > -1:
            if tl_wp > car_wp:
                rospy.loginfo('distance: %i', tl_wp - car_wp)
            else:
                rospy.loginfo('something went wrong, traffic light waypoint is behind us')


        #for pos in stop_line_positions:
        #    light_position = self.get_closest_waypoint(self.pose.pose)

        if light:
            state = self.get_light_state(light)
            return light_wp, state
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

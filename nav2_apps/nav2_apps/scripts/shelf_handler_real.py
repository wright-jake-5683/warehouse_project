import rclpy
import math
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav2_apps_interfaces.srv import GoToLoading
from nav2_apps.helpers.laser_manager_real import LaserManager
from nav2_apps.helpers.tf_manager import TfManager
from nav2_apps.helpers.robo_math import RoboMath
from nav2_apps.helpers.tf_manager import Transform
from nav2_apps.helpers.leg_data import LegData
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class ShelfHandler(Node):

    def __init__(self):
        super().__init__('shelf_handler_real')

        self.cb_group = ReentrantCallbackGroup()

        self.laser_helper_ = LaserManager()
        self.robo_math_helper_ = RoboMath()
        self.tf_manager_ = TfManager(self)
        self.cart_approach_complete_ = False


        # Publishers
        self.cmd_publisher_ = self.create_publisher(
            Twist,          
            "/cmd_vel",
            10               
        )

        self.lift_up_publisher_ = self.create_publisher(
            String,         
            "/elevator_up", 
            10               
        )

        self.lift_down_publisher_ = self.create_publisher(
            String,         
            "/elevator_down", 
            10               
        )

        self.laser_subscriber_ = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10,
            callback_group=self.cb_group  # ← allows laser to fire during service callback
        )

        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10,
            callback_group=self.cb_group # ← allows odom to fire during service callback
        )

        self.service_ = self.create_service(
            GoToLoading, '/approach_shelf', self.service_callback,
            callback_group=self.cb_group  # ← allows service to run alongside other callbacks
        )

        self.get_logger().info("Shelf Handler node started")
    
    def laser_callback(self, msg):
        self.front_laser_reading = self.laser_helper_.read_front_laser(msg)
        self.laser_data = msg


    def odom_callback(self, msg):
        self.odom_data = msg
        #self.get_logger().info(f"z: {self.odom_data.pose.pose.orientation.z}")

    def service_callback(self, request, response):
        try:
            self.get_logger().info("/approach shelf has been requested...")


            #while 1==1:
            #    for value in self.laser_data.intensities:
            #        if value > 4000:
            #            self.get_logger().info(f"{value} \n --------------------------")
            #    time.sleep(3)

            if request.attach_to_shelf:
                legs = self.detect_shelf_legs(self.laser_data)

                if not legs:
                    self.get_logger().info("Cannot detect shelf legs. Aborting task...")
                    response.complete = False
                    return response

                self.create_cart_frame(legs)

                while not self.cart_approach_complete_:
                    self.move_to_cart()

                self.center_under_cart()

                count = 0
                while count < 10:
                    self.get_logger().warn(f"Litfing cart, {10-count} second(s) left...")
                    msg = String()
                    self.lift_up_publisher_.publish(msg)
                    count += 1
                    time.sleep(1)

          
                response.complete = True

                #Back up
                velocity = self.robo_math_helper_.calculate_vel_by_distance(0.55, 5)
                start = time.monotonic()
                duration = 5.0  # seconds

                while time.monotonic() - start < duration:
                    msg = Twist()
                    msg.linear.x = -velocity * 2.5
                    self.cmd_publisher_.publish(msg)

            else:
                self.approach_shipping()
                self.get_logger().info("Approaching shipping position...")

                count = 0
                while count < 10:
                    self.get_logger().warn(f"Lowering cart, {10-count} second(s) left...")
                    msg = String()
                    self.lift_down_publisher_.publish(msg)
                    count += 1
                    time.sleep(1)

                self.slide_away_from_cart()
                response.complete = True

        except Exception as e:
            self.get_logger().error(f"Approach Shelf Service Exception: {e}")

        return response


    def approach_shipping(self):
        #Turn
        turn_complete = False
        while not turn_complete:
            delta_z = abs(self.odom_data.pose.pose.orientation.z) - 0.62
            #self.get_logger().info(f"delta_z: {delta_z}")
            if delta_z < .03:
                turn_complete = True
            else:
                msg = Twist()
                msg.angular.z = -0.25
                self.cmd_publisher_.publish(msg)

        #Move forward slightly
        velocity = self.robo_math_helper_.calculate_vel_by_distance(1.2, 7)
        start = time.monotonic()
        duration = 7.0  # seconds

        while time.monotonic() - start < duration:
            msg = Twist()
            msg.linear.x = velocity
            self.cmd_publisher_.publish(msg)


    def move_to_cart(self):
        rb1 = self.tf_manager_.get_tf_coords_parent_to_child("map", "robot_base_footprint")
        cart = self.tf_manager_.get_tf_coords_parent_to_child("map", "cart_frame")

        if not rb1 or not cart:
            self.get_logger().info("Either rb1 or cart frame couldn't be located")
            return

        dx = cart.x - rb1.x
        dy = cart.y - rb1.y
        distance = math.sqrt(dx**2 + dy**2)
        #self.get_logger().info(f"Distance from cart_frame: {round(distance * 100, 2)}cm")

        if distance > 0.195:
            msg = self.tf_manager_.move_subject_towards_target(rb1, cart)
            self.cmd_publisher_.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_publisher_.publish(msg)
            self.cart_approach_complete_ = True


    def center_under_cart(self):
        self.get_logger().info("RB1 in position, lining up with shelf legs...")

        # Ensure that RB1's yaw is in line with the cart_frame's yaw which was set to 0 when the frame was created
        tf = self.tf_manager_.get_tf_coords_parent_to_child("cart_frame", "robot_base_footprint")
        normalized_tf =  math.atan2(math.sin(tf.yaw), math.cos(tf.yaw))
        rb1_inline = normalized_tf < -1.495 and normalized_tf > -1.505
        #self.get_logger().info(f"cart to rb1 yaw: {normalized_tf}")

        msg = Twist()
        while not rb1_inline:
            tf = self.tf_manager_.get_tf_coords_parent_to_child("cart_frame", "robot_base_footprint")
            normalized_tf =  math.atan2(math.sin(tf.yaw), math.cos(tf.yaw))

            #self.get_logger().info(f"cart to rb1 yaw: {normalized_tf}")

            if normalized_tf > -1.495:
                msg.angular.z = -.1
    
            if normalized_tf < -1.505:
                msg.angular.z = .1
            
            #self.get_logger().warn(f"z: {msg.angular.z}")
            self.cmd_publisher_.publish(msg)

            rb1_inline = normalized_tf < -1.495 and normalized_tf > -1.505

        msg.angular.z = 0.0
        self.cmd_publisher_.publish(msg)        
        self.get_logger().info("RB1 is lined up with shelf, centering under shelf...")
            
        velocity = self.robo_math_helper_.calculate_vel_by_distance(0.425, 5)
        start = time.monotonic()
        duration = 5.0  # seconds

        while time.monotonic() - start < duration:
            msg = Twist()
            msg.linear.x = velocity
            self.cmd_publisher_.publish(msg)
        
        self.get_logger().info("RB1 is centered under shelf, lifting...")

    def slide_away_from_cart(self):
        self.get_logger().info("Sliding away from cart...")
        velocity = self.robo_math_helper_.calculate_vel_by_distance(0.85, 5)
        start = time.monotonic()
        duration = 5.0  # seconds

        while time.monotonic() - start < duration:
            msg = Twist()
            msg.linear.x = -velocity
            self.cmd_publisher_.publish(msg)
        self.get_logger().info("Sliding complete")


    def detect_shelf_legs(self, laser_data):
        self.get_logger().info("point hit")
        attempts = 0
        legs_detected = False
        while attempts < 3 and not legs_detected:  
            #for value in self.laser_data.intensities:
            #        if value > 3700:
            #           self.get_logger().info(f"v: {value} \n --------------------------")

            clusters = self.laser_helper_.cluster_laser_data(laser_data.intensities)
            self.get_logger().warn(f"clusters: {len(clusters)}")
            

            # Check if clusters is empty
            if not clusters or len(clusters) < 2:
                #Move forward slightly
                self.get_logger().warn("insufficent laser data to detect legs, moving RB1 forward slightly...")
                velocity = self.robo_math_helper_.calculate_vel_by_distance(0.075, 3)
                start = time.monotonic()
                duration = 3.0  # seconds

                msg = Twist()
                while time.monotonic() - start < duration:
                    msg.linear.x = velocity
                    self.cmd_publisher_.publish(msg)

                msg.linear.x = 0.0
                self.cmd_publisher_.publish(msg)

                attempts += 1
            else:
                legs_detected = True
        
        if not legs_detected:
            return []

        legs = []
        for cluster in clusters:
            middle_index = self.refine_cluster_center(cluster)
            legs.append(LegData(index=middle_index))

        for leg in legs:
            leg.distance = laser_data.ranges[leg.index]

        for leg in legs:
            leg.angle = self.laser_helper_.find_angle_from_laser_reading(laser_data, leg.index)

        for leg in legs:
            leg.point = self.robo_math_helper_.find_2d_coords_from_hypotenuse(leg.distance, leg.angle)

        return legs


    def refine_cluster_center(self, cluster, peak_fraction=0.7):
        """
        Dynamically threshold each cluster around its own peak intensity,
        then take the angular span midpoint of the surviving readings.
        """
        peak = max(reading.reading for reading in cluster)
        local_threshold = peak * peak_fraction

        filtered = [reading for reading in cluster if reading.reading >= local_threshold]
        if not filtered:
            filtered = cluster  # fallback to full cluster if nothing survives
        
        #for reading in filtered:
        #    self.get_logger().warn(f"filtered reading: {reading.reading}")
        #self.get_logger().info("-------------------------")

        # Strategy 1: angular span midpoint on the filtered readings
        first_index = filtered[0].index
        last_index = filtered[-1].index
        return (first_index + last_index) // 2

    def create_cart_frame(self, legs):
        midpoint = self.robo_math_helper_.find_midpoint(legs[0].point, legs[1].point)

        # 1. Transform midpoint from laser frame → odom frame
        point_in_laser = PointStamped()
        point_in_laser.header.frame_id = "robot_front_laser_base_link"
        point_in_laser.header.stamp = self.get_clock().now().to_msg()
        point_in_laser.point.x = midpoint.x
        point_in_laser.point.y = midpoint.y
        point_in_laser.point.z = 0.0

        point_in_odom = self.tf_manager_.transform_point(point_in_laser, "map")
        if point_in_odom is None:
            return

        # 2. Publish static transform with odom as parent
        new_transform = Transform(
            parent_frame="map",
            child_frame="cart_frame",
            translation_x=point_in_odom.point.x,
            translation_y=point_in_odom.point.y,
            translation_z=0.0,
            roll=0.0,
            pitch=0.0,
            yaw=0.0
        )

        self.tf_manager_.create_static_transform(new_transform)

def main(args=None):
    rclpy.init(args=args)
    node = ShelfHandler()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
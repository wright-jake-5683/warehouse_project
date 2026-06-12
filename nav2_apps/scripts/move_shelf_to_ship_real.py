import threading
import time

from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from nav2_apps_interfaces.srv import GoToLoading

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class ClientNode(Node):
    def __init__(self):
        super().__init__('approach_shelf_client_node')
        self.service_client = self.create_client(GoToLoading, '/approach_shelf')

    def call_service(self, task):
        task = task.lower()
        # Wait for the service to be available
        if not self.service_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/approach_shelf Service is not available')
            return None

        # Build and send the request
        request = GoToLoading.Request()
        if task == "lift":
            request.attach_to_shelf = True
        else:
            request.attach_to_shelf = False

        future = self.service_client.call_async(request)
        # need to spin navigator node here to make service call bc BasicNavigator only spins when a specific
        # internal function like goToPose(), etc, is called
        #rclpy.spin_until_future_complete(self, future) 
        while not future.done():
            time.sleep(0.05)

        if future.result() is not None:
            return future.result().complete
        else:
            raise RuntimeError('Service call failed')



shelf_footprint = "[ [-0.425, -0.4], [0.425, -0.4], [0.425, 0.4], [-0.425, 0.4] ]"
robot_footprint = "[ [0.15, 0.0], [-0.105, 0.105], [-0.105, -0.105] ]"

costmap_nodes = [
    '/local_costmap/local_costmap',
    '/global_costmap/global_costmap'
]

def set_costmap_params(node, params: dict):
    """
        params: dict of {param_name: (ParameterType, value)}
        e.g. {'footprint': (ParameterType.PARAMETER_STRING, '[[...]]'),
            'robot_radius': (ParameterType.PARAMETER_DOUBLE, 0.15)}
    """
    param_list = []
    for name, (ptype, value) in params.items():
        pv = ParameterValue(type=ptype)
        if ptype == ParameterType.PARAMETER_STRING:
            pv.string_value = value
        elif ptype == ParameterType.PARAMETER_DOUBLE:
            pv.double_value = value
        param_list.append(Parameter(name=name, value=pv))

    for costmap_node in costmap_nodes:
        client = node.create_client(SetParameters, f'{costmap_node}/set_parameters')

        if not client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error(f'{costmap_node} set_parameters not available')
            continue

        request = SetParameters.Request()
        request.parameters = param_list

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            node.get_logger().info(f'Parameters updated on {costmap_node}')
        else:
            node.get_logger().error(f'Failed to update parameters on {costmap_node}')

def switch_to_shelf_footprint(node):
    """
        Switch from robot_radius to polygon footprint for carrying shelf.
    """
    print('Switching to shelf footprint...')
    set_costmap_params(node, {
        'footprint': (
            ParameterType.PARAMETER_STRING,
            shelf_footprint
        )
    })


def switch_to_robot_footprint(node):
    """
        Switch back to robot_radius after dropping shelf.
    """
    print('Restoring robot radius...')
    set_costmap_params(node, {
        'footprint': (
            ParameterType.PARAMETER_STRING,
            robot_footprint  # triangle approximating robot_radius circle
        )
    })



def go_to_loading_zone_task(client_node, navigator, positions, initial_pose):
    loading_zone_pose = PoseStamped()
    loading_zone_pose.header.frame_id = 'map'
    loading_zone_pose.header.stamp = navigator.get_clock().now().to_msg()
    loading_zone_pose.pose.position.x = positions['loading_position'][0]
    loading_zone_pose.pose.position.y = positions['loading_position'][1]
    loading_zone_pose.pose.orientation.z = positions['loading_position'][2]
    loading_zone_pose.pose.orientation.w = positions['loading_position'][3]
    print('Received request to go to loading zone...')
    navigator.goToPose(loading_zone_pose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival to loading zone ' +
                  '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('RB1 has arrived at the loading zone. Moving RB1 under shelf...')
        success = client_node.call_service("lift")
        
        if not success:
            print('Failed to attach to shelf!')
            exit(1)
        
        switch_to_shelf_footprint(navigator)
        # Clear stale costmap data
        navigator.clearLocalCostmap()

        # Give the costmap time to repopulate with current sensor data
        time.sleep(1.0)  # adjust based on your costmap update_frequency
        return True

    elif result == TaskResult.CANCELED:
        print("RB1's current task has been canceled, returning to initial position...")
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)
        return False

    elif result == TaskResult.FAILED:
        print('RB1 has failed to reach loading zone, shutting down...')
        exit(1)


def go_to_shipping_zone_task(client_node, navigator, positions, initial_pose):
    shipping_zone_pose = PoseStamped()
    shipping_zone_pose.header.frame_id = 'map'
    shipping_zone_pose.header.stamp = navigator.get_clock().now().to_msg()
    shipping_zone_pose.pose.position.x = positions['shipping_position'][0]
    shipping_zone_pose.pose.position.y = positions['shipping_position'][1]
    shipping_zone_pose.pose.orientation.z = positions['shipping_position'][2]
    shipping_zone_pose.pose.orientation.w = positions['shipping_position'][3]
    print('Received request to go to shipping zone...')
    navigator.goToPose(shipping_zone_pose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival to shipping zone ' +
                  '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('RB1 has arrived at the shipping zone. Releasing shelf...')
        #success = client_node.call_service("drop")

        #if not success:
        #    print("Failed to drop shelf!")
        #    exit(1)

        switch_to_robot_footprint(navigator)
        return True

    elif result == TaskResult.CANCELED:
        print("RB1's current task has been canceled, returning to initial position...")
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)
        return False

    elif result == TaskResult.FAILED:
        print('RB1 has failed to reach shipping zone, shutting down...')
        exit(1)

def go_to_initial_pose(navigator, initial_pose):
    navigator.goToPose(initial_pose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival to initial position ' +
                  '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("RB1 has arrived at it's initial position")
        return True

    elif result == TaskResult.CANCELED:
        print("RB1's return to it's initial position has been cancled...")
        return False

    elif result == TaskResult.FAILED:
        print("RB1 has failed to reach it's intial position, shutting down...")
        exit(1)



def main():
    positions = {
        "loading_position": [4.65, -0.457, -0.907543, 0.70667],
        "shipping_position": [2.33, 0.10, 0.702085, 0.70667]
    }

    rclpy.init()

    navigator = BasicNavigator()
    client_node = ClientNode()

    # Run both nodes in a MultiThreadedExecutor in a background thread
    executor = MultiThreadedExecutor()
    executor.add_node(navigator)
    executor.add_node(client_node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Set your demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.000
    initial_pose.pose.position.y = 0.10
    initial_pose.pose.orientation.z = 0.000
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)
    navigator.get_logger().info(f"Initial Pose set at: x:{initial_pose.pose.position.x}, y:{initial_pose.pose.position.y}")

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()
    navigator.get_logger().info("Navigator node is fully active")

    loaded = go_to_loading_zone_task(client_node, navigator, positions, initial_pose)
    if not loaded:
        exit(1)

    #shipped = go_to_shipping_zone_task(client_node, navigator, positions, initial_pose)
    #if not shipped:
    #    print("Was unable to carry shelf to shipping. Returning to initial position...")

    #print("Navigating back to RB1's initial position...")
    #reset_complete = go_to_initial_pose(navigator, initial_pose)
    #if reset_complete:
    #    print("Ready for next task")

    exit(0)


if __name__ == '__main__':
    main()
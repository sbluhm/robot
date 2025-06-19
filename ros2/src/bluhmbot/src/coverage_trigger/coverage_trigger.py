#!/usr/bin/env python3

import cv2 as cv #read the image
import rclpy
import time
from action_msgs.msg import GoalStatus
from enum import Enum
from geometry_msgs.msg import Point32, Polygon, TransformStamped
from lifecycle_msgs.srv import GetState
from opennav_coverage_msgs.action import NavigateCompleteCoverage
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3

def is_point_in_polygon(x, y, polygon):
    """
    Determine if a point is inside a polygon using the ray casting algorithm.

    Args:
        x, y: Coordinates of the point.
        polygon: List of (x, y) tuples representing the polygon's vertices.

    Returns:
        True if the point is inside the polygon, False otherwise.
    """
    num = len(polygon)
    j = num - 1
    inside = False

    for i in range(num):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)) and \
            (x < (xj - xi) * (y - yi) / (yj - yi + 1e-10) + xi):
            inside = not inside
        j = i

    return inside

def getContour(x, y):
    img = cv.imread("/root/robot/ros2/src/bluhmbot/maps/house2.png")
    mapshiftx = -7.14
    mapshifty = -7.83
    height, width, channels = img.shape

#convert the image to grayscale
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
#blur image to reduce the noise in the image while thresholding. #This smoothens the sharp edges in the image.
    blur = cv.blur(gray, (10,10))
#Apply thresholding to the image
    ret, thresh = cv.threshold(blur, 1, 255, cv.THRESH_OTSU)
#find the contours in the image
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    polygon = 0
    print(f"Length Contours: {len(contours)}")
    for i in range(len(contours)):
        if is_point_in_polygon((x-mapshiftx)/0.007229358,(y-mapshifty)/0.007229358, contours[i].reshape(-1, 2)):
            polygon = i
            break

    new = (contours[polygon].reshape(-1, 2)*0.007229358).tolist()
    originx = -7.14
    originy = -7.83
    flipped = [[x+originx, -y+height*0.0072293588+originy] for x, y in new]
    field = flipped
    field.append(field[0])
    return field


class CoverageNavigatorTester(Node):

    def __init__(self):
        super().__init__(node_name='coverage_navigator_tester')
        self.goal_handle = None
        self.result_future = None
        self.status = None
        self.feedback = None

        self.coverage_client = ActionClient(self, NavigateCompleteCoverage,
                                            'navigate_complete_coverage')

    def destroy_node(self):
        self.coverage_client.destroy()
        super().destroy_node()

    def toPolygon(self, field):
        poly = Polygon()
        for coord in field:
            pt = Point32()
            pt.x = coord[0]
            pt.y = coord[1]
            poly.points.append(pt)
        return poly

    def navigateCoverage(self, field):
        """Send a `NavToPose` action request."""
        print("Waiting for 'NavigateCompleteCoverage' action server")
        while not self.coverage_client.wait_for_server(timeout_sec=1.0):
            print('"NavigateCompleteCoverage" action server not available, waiting...')

        goal_msg = NavigateCompleteCoverage.Goal()
        goal_msg.frame_id = 'map'
        goal_msg.polygons.append(self.toPolygon(field))

        print('Navigating to with field of size: ' + str(len(field)) + '...')
        send_goal_future = self.coverage_client.send_goal_async(goal_msg,
                                                                self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            print('Navigate Coverage request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def isTaskComplete(self):
        """Check if the task request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                print(f'Task with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        print('Task succeeded!')
        return True

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def startup(self, node_name='bt_navigator'):
        # Waits for the node within the tester namespace to become active
        print(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            print(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            print(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                print(f'Result of get_state: {state}')
            time.sleep(2)
        return


def Navigate(field):
    navigator = CoverageNavigatorTester()
    navigator.startup()
    navigator.navigateCoverage(field)

    i = 0
    while not navigator.isTaskComplete():
        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')
        time.sleep(1)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')


class PositionLookup(Node):
    def __init__(self):
        super().__init__('position_lookup')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

def main():
    rclpy.init()
    node = PositionLookup()
    try:
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < 5:
            rclpy.spin_once(node, timeout_sec=1.0)
            try:
                target_frame="map"
                source_frame="base_footprint"
                now = rclpy.time.Time()
                if node.tf_buffer.can_transform(target_frame, source_frame, now):
                    trans: TransformStamped = node.tf_buffer.lookup_transform(target_frame, source_frame, now)
                    node.get_logger().info(f"Robot position: {trans.transform.translation}")
                    field = getContour(trans.transform.translation.x, trans.transform.translation.y)
                    Navigate(field)
                    break
        
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                node.get_logger().warn(f"Transform not available: {e}")        
                pass

        else:
            node.get_logger().warn(f"Transform from '{source_frame}' to '{target_frame}' not available after {timeout_sec} seconds.")

    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


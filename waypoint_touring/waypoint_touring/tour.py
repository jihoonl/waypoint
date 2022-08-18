#!/usr/bin/env python

import random
import rclpy
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.action import ActionClient
import waypoint_touring.utils as utils

import nav2_msgs.action as nav2_actions
import visualization_msgs.msg as viz_msgs


class TourMachine(Node):
    """
    Touring the given waypoints
    """

    def __init__(self):
        super().__init__('tour_machine')

        self._declare_parameters()

        filename = self.get_parameter('filename').value
        repeat = self.get_parameter('repeat').value
        random_visits = self.get_parameter('random_visits').value

        action_name = 'navigate_to'
        self._ac_move_base = ActionClient(self, nav2_actions.NavigateToPose,
                                          action_name)
        self.get_logger().info('Wait for %s server' % action_name)
        #self._ac_move_base.wait_for_server()

        # Load waypoints from yaml file
        self._waypoints = utils.get_waypoints(filename)
        self._counter = 0
        self._repeat = repeat
        self._random_visits = random_visits

        # Shuffling the waypoints for a tour
        if self._random_visits:
            random.shuffle(self._waypoints)

        # Set visualization marker publisher as latched
        # Ref: https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html#comparison-to-ros-1
        latching_qos = QoSProfile(depth=1,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_viz_marker = self.create_publisher(viz_msgs.MarkerArray,
                                                     'viz_waypoints',
                                                     qos_profile=latching_qos)
        self._viz_markers = utils.create_viz_markers(self._waypoints)

    def _declare_parameters(self):
        self.declare_parameter('filename', 'waypoints.yaml')
        self.declare_parameter('repeat', False)
        self.declare_parameter('random_visits', False)

    def move_to_next(self):
        p = self._get_next_destination()
        if not p:
            self.get_logger().info("Finishing Tour")
            return True

        goal = utils.create_navigate_to_pose_goal(p, self.get_clock().now())
        self.get_logger().info("Move to %s" % p['name'])
        self._ac_move_base.send_goal(goal)
        self._ac_move_base.wait_for_result()
        result = self._ac_move_base.get_result()
        rospy.loginfo("Result : %s" % result)

        return False

    def _get_next_destination(self):
        if self._counter == len(self._waypoints):
            if self._repeat:
                self._counter = 0
                if self._random_visits:
                    random.shuffle(self._waypoints)
            else:
                next_destination = None
        next_destination = self._waypoints[self._counter]
        self._counter = self._counter + 1
        return next_destieation

    def spin(self):
        print('here')
        rate = self.create_rate(1)
        print('here2')
        rate.sleep()
        # Publish visualization markers before string a tour
        print('here3')
        self._pub_viz_marker.publish(self._viz_markers)
        finished = False
        try:
            while rclpy.ok() and not finished:
                #finished = self.move_to_next()
                rate.sleep()
        except KeyboardInterrupt:
            pass


def main(args=None):
    rclpy.init(args=args)

    m = TourMachine()
    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(m, ), daemon=True)
    thread.start()

    m.spin()
    rclpy.shutdown()
    thread.join()



if __name__ == '__main__':
    main()

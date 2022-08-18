#!/usr/bin/env python

import yaml
import rclpy
from rclpy.node import Node
import geometry_msgs.msg as geometry_msgs


class WaypointGenerator(Node):

    def __init__(self):
        super().__init__('waypoint_generator')
        self._sub_pose = self.create_subscription(geometry_msgs.PointStamped,
                                                  'clicked_point',
                                                  self._process_pose, 1)
        self._sub_pose
        self.declare_parameter('filename', 'waypoints.yaml')

        self._waypoints = []
        self._filename = self.get_parameter('filename').value
        self.get_logger().info(str(self._filename))

    def _process_pose(self, msg):
        p = msg.point

        data = {}
        data['frame_id'] = msg.header.frame_id
        data['pose'] = {}
        data['pose']['position'] = {'x': p.x, 'y': p.y, 'z': 0.0}
        data['pose']['orientation'] = {'x': 0, 'y': 0, 'z': 0, 'w': 1}
        data['name'] = '%s_%s' % (p.x, p.y)

        self._waypoints.append(data)
        self.get_logger().info("Clicked : (%s, %s, %s)" % (p.x, p.y, p.z))

    def write_file(self):
        ways = {}
        ways['waypoints'] = self._waypoints
        with open(self._filename, 'w') as f:
            f.write(yaml.dump(ways, default_flow_style=False))


def main(args=None):
    rclpy.init(args=args)
    g = WaypointGenerator()
    g.get_logger().info('Initialized')
    try:
        rclpy.spin(g)
    except KeyboardInterrupt:
        pass
    g.write_file()
    g.get_logger().info('Bye Bye')


if __name__ == '__main__':
    main()

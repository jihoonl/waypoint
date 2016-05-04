#!/usr/bin/env python

import random
import rospy
import actionlib
import waypoint_touring.utils as utils

import move_base_msgs.msg as move_base_msgs
import visualization_msgs.msg as viz_msgs

class TourMachine(object):

    def __init__(self, filename, random_visits=False, repeat=False):
        self._waypoints = utils.get_waypoints(filename)

        action_name = 'move_base'
        self._ac_move_base = actionlib.SimpleActionClient(action_name, move_base_msgs.MoveBaseAction)
        rospy.loginfo('Wait for %s server' % action_name)
        self._ac_move_base.wait_for_server
        self._counter = 0
        self._repeat = repeat
        self._random_visits = random_visits

        if self._random_visits:
            random.shuffle(self._waypoints)
        self._pub_viz_marker = rospy.Publisher('viz_waypoints', viz_msgs.MarkerArray, queue_size=1, latch=True)
        self._viz_markers = utils.create_viz_markers(self._waypoints)

    def move_to_next(self):
        p = self._get_next_destination()

        if not p:
            rospy.loginfo("Finishing Tour")
            return True

        goal = utils.create_move_base_goal(p)
        rospy.loginfo("Move to %s" % p['name'])
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
        return next_destination

    def spin(self):
        rospy.sleep(1.0)
        self._pub_viz_marker.publish(self._viz_markers)
        finished = False
        while not rospy.is_shutdown() and not finished:
            finished = self.move_to_next()
            rospy.sleep(2.0)

if __name__ == '__main__':
    rospy.init_node('tour')

    filename = rospy.get_param('~filename')
    random_visits = rospy.get_param('~random', False)
    repeat = rospy.get_param('~repeat', False)

    m = TourMachine(filename, random_visits, repeat)
    rospy.loginfo('Initialized')
    m.spin()
    rospy.loginfo('Bye Bye')

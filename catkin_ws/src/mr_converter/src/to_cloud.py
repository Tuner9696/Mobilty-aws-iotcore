#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity
import time
import csv
import math
import rospy
import tf
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionResult
from mr_converter.msg import AwsRoboCurPosUpdate
from mr_converter.msg import AwsRoboCurStatusUpdate


class RobotStatePublisher:
    STATUS_MAP = [
        'PENDING',
        'ACTIVE',
        'PREEMPTED',
        'SUCCEEDED',
        'ABORTED',
        'REJECTED',
        'PREEMPTING',
        'RECALLING',
        'RECALLED',
        'LOST'
    ]
    local_to_global = []

    def __init__(self):
        rospy.init_node('MR-Converter_to_Cloud', anonymous=True)
        self.from_robot_id = rospy.get_param('~from_robot_id')
        self.to_robot_id = rospy.get_param('~to_robot_id')
        self.to_global_matrix = rospy.get_param('~to_global_matrix')

        csv_file= open(self.to_global_matrix , "r")
        f = csv.reader(csv_file, delimiter=",", doublequote=True, lineterminator="\r\n", quotechar='"', skipinitialspace=True)
        line_1 = next(f)
        line_2 = next(f)
        csv_file.close()
        line_1_float = [float(str) for str in line_1]
        line_2_float = [float(str) for str in line_2]
        self.local_to_global.append(line_1_float)
        self.local_to_global.append(line_2_float)
        # 計算誤差を考慮
        if self.local_to_global[0][0] > 1:
            self.local_to_global[0][0] = 1
        elif self.local_to_global[0][0] < -1:
            self.local_to_global[0][0] = -1

        #rospy.Subscriber(self.from_robot_id + '/move_base/result', MoveBaseActionResult, self.__publish_status)
        rospy.Subscriber(self.from_robot_id + '/move_base/status', GoalStatusArray, self.__publish_status)
        self.pos_pub = rospy.Publisher('/aws/position', AwsRoboCurPosUpdate, queue_size=1)
        self.status_pub = rospy.Publisher('/aws/status', AwsRoboCurStatusUpdate, queue_size=1)

        self.refresh_rate = rospy.Rate(2)
        self.listener = tf.TransformListener()
        self.curr_status = -1

    def execute(self):
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform('/map', self.from_robot_id+'_tf/base_footprint', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue;
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
            self.__publish_position(trans, pitch, roll, yaw)
            self.refresh_rate.sleep()

    @staticmethod
    def __stamp_to_timestamp(stamp):

        timestamp = stamp.secs + stamp.nsecs / 1000000000.
        return time.time()

    def __publish_position(self, trans, pitch, roll, yaw):
        status = AwsRoboCurPosUpdate()
        status.state.desired.timestamp = time.time()

        status.state.desired.x = round(self.local_to_global[0][0] * trans[0] + self.local_to_global[0][1] * trans[1] + self.local_to_global[0][2], 6)
        status.state.desired.y = round(self.local_to_global[1][0] * trans[0] + self.local_to_global[1][1] * trans[1] + self.local_to_global[1][2], 6)
        status.state.desired.z = trans[2]
        status.state.desired.p = pitch
        status.state.desired.r = roll
        if self.local_to_global[1][0] > 0:
            status.state.desired.w = yaw + math.acos(self.local_to_global[0][0])
        else:
            status.state.desired.w = yaw + (2 * math.pi - math.acos(self.local_to_global[0][0]))
        if status.state.desired.w > math.pi:
            status.state.desired.w = status.state.desired.w - 2 * math.pi
        status.state.desired.vehicle_id = self.to_robot_id
        self.pos_pub.publish(status)

    def __publish_status(self, msg):
        updated = False
        if type(msg) is MoveBaseActionResult:
            self.curr_status = int(msg.status.status)
            updated = True
        elif type(msg) is GoalStatusArray:
            cnt = len(msg.status_list)
            if cnt > 0:
                tmp_status = msg.status_list[cnt-1].status
                if tmp_status != self.curr_status:
                    self.curr_status = tmp_status
                    updated = True
        if updated:
            timestamp = RobotStatePublisher.__stamp_to_timestamp(msg.header.stamp)

            status = AwsRoboCurStatusUpdate()
            status.state.desired.timestamp = timestamp
            status.state.desired.vehicle_id = self.to_robot_id
            status.state.desired.status = RobotStatePublisher.STATUS_MAP[self.curr_status]
            self.status_pub.publish(status)


if __name__ == '__main__':

    state_publisher = RobotStatePublisher()
    try:
        state_publisher.execute()
    except Exception as e:
        print(e)

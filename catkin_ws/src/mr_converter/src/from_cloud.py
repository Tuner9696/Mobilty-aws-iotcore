#!/usr/bin/env python

import time
import csv
import math
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from mr_converter.msg import AwsRoboDestPosUpdate, AwsCommand, AwsUpdatePosition, AwsUpdateOrientation
from actionlib_msgs.msg import GoalID
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


class Commander:
    global_to_local = []

    @staticmethod
    def __convert_id(vehicle_id):
        if vehicle_id.split('_')[0] == "temi":
            return "robot1"
        else:
            return "robot2"

    def __command_callback(self, msg):
        vehicle_ids = msg.vehicle_ids.split(' ')
        goal_id = GoalID()
        for vehicle_id in vehicle_ids:
            command = msg.command.split(' ')[0]
            if vehicle_id not in self.publisher_map or command not in self.publisher_map[vehicle_id]:
                to_vehicle_id = self.__convert_id(vehicle_id)
                if command == 'stop':
                    publisher = rospy.Publisher('/'+to_vehicle_id+'/move_base/cancel',
                                                GoalID, queue_size=1, latch=True)
                elif command == 'move_to':
                    publisher = rospy.Publisher('/'+to_vehicle_id+'/initialpose',
                                                PoseWithCovarianceStamped, queue_size=1, latch=True)

                self.publisher_map[vehicle_id] = {command: publisher}
            else:
                publisher = self.publisher_map[vehicle_id][command]
            if command == 'stop':
                goal_id.id = ""
                goal_id.stamp.secs = 0.0
                goal_id.stamp.nsecs = 0.0
                publisher.publish(goal_id)
            elif command == 'move_to':
                position = AwsUpdatePosition()
                orientation = AwsUpdateOrientation()
                args = msg.command.split(' ')[1:4]
                position.x = float(args[0]); position.y = float(args[1]); orientation.w = float(args[2])

                state_msg = ModelState()
                state_msg.model_name = to_vehicle_id.replace('robot', 'Robot')
                state_msg.pose.position.x = position.x
                state_msg.pose.position.y = position.y
                state_msg.pose.position.z = 0
                q = tf.transformations.quaternion_from_euler(0.0, 0.0, orientation.w)
                state_msg.pose.orientation.x = q[0]
                state_msg.pose.orientation.y = q[1]
                state_msg.pose.orientation.z = q[2]
                state_msg.pose.orientation.w = q[3]

                rospy.wait_for_service('/gazebo/set_model_state')
                try:
                    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                    resp = set_state(state_msg)

                except rospy.ServiceException as e:
                    print("Service call failed: %s" % e)

                # wait for stabilizing of position estimation for the new location in 3D space.
                time.sleep(10)

                pose = PoseWithCovarianceStamped()
                pose.pose.pose = self.__transform(position, orientation, self.global_to_local)
                pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.get_rostime()
                publisher.publish(pose)


    def __transform(self, position, orientation, matrix):
        pose = Pose()
        pose.position.x = round(float(matrix[0][0]) * position.x + float(matrix[0][1]) * position.y + float(matrix[0][2]), 6)
        pose.position.y = round(float(matrix[1][0]) * position.x + float(matrix[1][1]) * position.y + float(matrix[1][2]), 6)
        if float(matrix[1][0]) > 0:
            orientation.w = orientation.w + math.acos(float(matrix[0][0]))
        else:
            orientation.w = orientation.w + (2 * math.pi - math.acos(float(matrix[0][0])))
        if orientation.w > math.pi:
            orientation.w = orientation.w - 2 * math.pi
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, orientation.w)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose

    def __goal_callback(self, msg):
        pose = PoseStamped()

        from_vehicle_id = msg.state.desired.vehicle_id
        print(from_vehicle_id)
        if from_vehicle_id not in self.goal_publisher_map:
            to_vehicle_id = self.__convert_id(from_vehicle_id)
            publisher = rospy.Publisher('/' + to_vehicle_id + '/move_base_simple/goal',
                                        PoseStamped, queue_size=1, latch=True)
            self.goal_publisher_map[from_vehicle_id] = publisher
        else:
            publisher = self.goal_publisher_map[from_vehicle_id]
        pose.header.frame_id = msg.state.desired.frame_id
        pose.pose = self.__transform(msg.state.desired.position, msg.state.desired.orientation, self.global_to_local)
        publisher.publish(pose)

    def __init__(self):
        self.publisher_map = {}
        rospy.init_node('MR-Converter_from_Cloud', anonymous=True)
        self.to_local_matrix = rospy.get_param('~to_local_matrix')
        csv_file= open(self.to_local_matrix, "r")
        f = csv.reader(csv_file, delimiter=",", doublequote=True, lineterminator="\r\n", quotechar='"',
                       skipinitialspace=True)
        line_1 = next(f)
        line_2 = next(f)
        csv_file.close()
        self.make_matrix(line_1, line_2)
        self.goal_publisher_map = {}
        self.cancel_publisher_map = {}

    def make_matrix(self, line_1, line_2):
        line_1_float = [float(str) for str in line_1]
        line_2_float = [float(str) for str in line_2]
        self.global_to_local.append(line_1_float)
        self.global_to_local.append(line_2_float)
        if self.global_to_local[0][0] > 1:
            self.global_to_local[0][0] = 1
        elif self.global_to_local[0][0] < -1:
            self.global_to_local[0][0] = -1

    def execute(self):
        rospy.Subscriber('/aws/move', AwsRoboDestPosUpdate, self.__goal_callback)
        rospy.Subscriber('/aws/command', AwsCommand, self.__command_callback)
        rospy.spin()


if __name__ == "__main__":
    commander = Commander()
    try:
        commander.execute()
    except rospy.ROSInterruptException:
        pass

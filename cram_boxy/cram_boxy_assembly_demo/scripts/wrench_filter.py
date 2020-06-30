#!/usr/bin/env python

import numpy as np
from std_msgs.msg import String
import rospy
from geometry_msgs.msg import WrenchStamped, Vector3


class Filter(rospy.Subscriber):
    def __init__(self, pub, topic, message_type):
        self.pub = pub
        self.data_buffer = []
        self.data_mean = WrenchStamped()
        self.mean_array = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        super(Filter, self).__init__(topic, message_type, self.callback)

    def callback(self, data):
        nforce = data.wrench.force
        ntorque = data.wrench.torque
        self.mean_array[0] = self.mean_array[0] * 0.975 + np.array([nforce.x, nforce.y, nforce.z]) * 0.025
        self.mean_array[1] = self.mean_array[1] * 0.975 + np.array([ntorque.x, ntorque.y, ntorque.z]) * 0.025
        response = WrenchStamped()
        response.wrench.force = Vector3(self.mean_array[0][0], self.mean_array[0][1], self.mean_array[0][2])
        response.wrench.torque = Vector3(self.mean_array[1][0], self.mean_array[1][1], self.mean_array[1][2])
        self.pub.publish(response)

    # def callback(self, data):
    #     self.data_buffer.append(data)
    #     if len(self.data_buffer) > 10:
    #         self.data_buffer.remove(self.data_buffer[0])
    #         force = sum(numpy.array([a.wrench.force.x, a.wrench.force.y, a.wrench.force.z]) for a in self.data_buffer) * 0.1
    #         torque = sum(numpy.array([a.wrench.torque.x, a.wrench.torque.y, a.wrench.torque.z]) for a in self.data_buffer) * 0.1
    #         self.data_mean.wrench.force.x = force[0]
    #         self.data_mean.wrench.force.y = force[1]
    #         self.data_mean.wrench.force.z = force[2]
    #         self.data_mean.wrench.torque.x = torque[0]
    #         self.data_mean.wrench.torque.y = torque[1]
    #         self.data_mean.wrench.torque.z = torque[2]
    #         self.pub.publish(self.data_mean)


def listener():
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher("left_arm_kms40/wrench_filtered", WrenchStamped, queue_size=10)
    Filter(pub, "left_arm_kms40/wrench", WrenchStamped)
    rospy.spin()


if __name__ == '__main__':
    listener()
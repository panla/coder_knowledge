#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
AddTwoInts.msg

int64 x
int64 y


AddTwoIntsResponse
int64 sum

<https://wiki.ros.org/cn/ROS/Tutorials/Recording%20and%20playing%20back%20data>
<https://wiki.ros.org/cn/ROS/Tutorials/WritingPublisherSubscriber%28python%29>
"""

import rospy

from project_msgs.msg import String


class Publisher:
    """
    # 加上 taler.py
    catkin_install_python(PROGRAMS scripts/talker.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    """

    def talker(self):

        # 定义发布者
        publisher = rospy.Publisher('chatter', String, queue_size=10)

        # 初始化节点
        rospy.init_node('talker', anonymous=True)

        # 定义 10Hz 的频率
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # 如果节点未终止

            # 定义一个字符串，带上时间
            info = "Hello_world {0}".format(rospy.get_time())

            # info = String()
            # info.data = ""

            # 使用日志打印出来，Level=INFO
            rospy.loginfo(info)

            # 发布出去
            publisher.publish(info)

            rate.sleep()


class Subscriber:

    """
    # 再加上 listener.py
    catkin_install_python(PROGRAMS scripts/talker.py scripts/listener.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    """
    def sub_callback(data):

        rospy.loginfo(rospy.get_caller_id() + "I heard {0}".format(data.data))

    def talker(self):
        # 定义订阅者
        rospy.Subscriber('chatter', String, self.sub_callback)

        # 初始化节点
        rospy.init_node('talker', anonymous=True)

        rospy.spin()


class ServiceServer:

    """
    catkin_install_python(PROGRAMS scripts/add_two_ints_server.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    """

    def handle_add_two_ints(req):
        print(req.x, req.y)

        return AddTwoIntsResponse(req.x + req.y)

    def add_two_ints_server(self):

        # 定义服务
        rospy.Service('add_two_ints', AddTwoInts, self.handle_add_two_ints)

        # 初始化
        rospy.init_node('add_two_ints_server')

        rospy.spin()


class ServiceClient:

    """
    catkin_install_python(PROGRAMS scripts/add_two_ints_server.py scripts/add_two_ints_client.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    """

    def add_two_ints(self, x, y):

        # 在add_two_ints服务可用之前一直阻塞
        rospy.wait_for_service('add_two_ints')

        try:
            # 为服务的调用创建了一个句柄
            add_two_ints_client = rospy.ServiceProxy('add_two_ints', AddTwoInts)

            response = add_two_ints_client(10, 20)

        except rospy.ServiceException as exc:
            rospy.logerr(str(exc))


if __name__ == '__main__':

    try:
        Publisher().talker()
        Subscriber().talker()

        ServiceServer().add_two_ints_server()

    except rospy.ROSInterruptException:
        pass

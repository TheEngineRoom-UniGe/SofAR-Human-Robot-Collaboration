#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService
from baxter_moveit.msg import BaxterMoveitJoints, BaxterTrajectory
from baxter_moveit.srv import MoverService, TrajectoryService
from geometry_msgs.msg import Quaternion, Pose
from human_baxter_collaboration.msg import HumanTf


def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    tcp_server = TcpServer(ros_node_name)
    rospy.init_node(ros_node_name, anonymous=True)

    # Start the Server Endpoint with a ROS communication objects dictionary for routing messages
    tcp_server.start({
    	'human_tf': RosPublisher('human_tf', HumanTf, queue_size=100),
        'baxter_moveit': RosService('baxter_moveit', MoverService),
        'baxter_moveit_trajectory': RosService('baxter_moveit_trajectory', TrajectoryService)
    })
    
    rospy.spin()


if __name__ == "__main__":
    main()

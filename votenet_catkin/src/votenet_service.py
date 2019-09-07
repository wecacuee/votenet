#!/usr/bin/python3
import numpy as np

import rospy
from votenet_catkin.srv import VotenetResponse, Votenet
from ros_numpy.point_cloud2 import pointcloud2_to_array, array_to_pointcloud2

from demo import point_cloud_to_detections


def votenet_callback(req):
    points = pointcloud2_to_array(req.pc)
    result_points, pred_map_cls = point_cloud_to_detections(points)
    pc = array_to_pointcloud2(
        np.asarray(result_points, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')]))
    msg = VotenetResponse(pc=pc)
    return msg


def votenet_server():
    rospy.init_node('Votenet_server')
    s = rospy.Service('Votenet', Votenet, votenet_callback)
    rospy.spin()


if __name__ == '__main__':
    votenet_server()

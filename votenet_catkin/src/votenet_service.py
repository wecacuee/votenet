#!/usr/bin/python3
import rospy
import votenet.srv
from ros_numpy.point_cloud2 import pointcloud2_to_array, array_to_pointcloud2

from demo import point_cloud_to_detections

def votenet_callback(req):
    points = pointcloud2_to_array(req.pc)
    result_points, pred_map_cls = point_cloud_to_detections(points)
    return votenet.srv.VotenetResponse(pc=array_to_pointcloud2(result_points))


def votenet_server():
    rospy.init_node('Votenet_server')
    s = rospy.Service('Votenet', votenet.srv.Votenet, votenet_callback)
    rospy.spin()


if __name__ == '__main__':
    votenet_server()

#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

class RvizMarkerPublisher:
    def __init__(self):
        # Inicializar el nodo
        #rospy.init_node('rviz_marker_publisher', anonymous=True)

        # Crear un publicador para el marcador
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        # Crear un objeto Marker
        self.marker = Marker()
        self.marker.header.frame_id = "robot1_tf/odom_groundtruth"
        self.marker.ns = "basic_shapes"
        self.marker.id = 0
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.2
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.2
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0

    def set_marker_position(self, x, y, z):
        # Establecer las coordenadas del marcador
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z

    def run(self):
        # Bucle de publicación
        rate = rospy.Rate(1)  # Publicar a 1 Hz
        while not rospy.is_shutdown():
            self.marker.header.stamp = rospy.Time.now()
            self.marker_pub.publish(self.marker)
            rate.sleep()

if __name__ == '__main__':
    try:
        marker_publisher = RvizMarkerPublisher()
        marker_publisher.set_marker_position(1.0, 2.0, 0.0)  # Establecer las coordenadas aquí
        marker_publisher.run()
    except rospy.ROSInterruptException:
        pass

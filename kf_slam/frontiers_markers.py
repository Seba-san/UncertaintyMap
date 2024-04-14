import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


def clear_all_markers(publisher,frame_id="map"):
    
    clear_marker = MarkerArray()
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "borders"
    marker.action = Marker.DELETEALL
    clear_marker.markers.append(marker)
    publisher.publish(clear_marker)

def create_border_marker(centers, frame_id="map", marker_id=0, radius=1.0,id=0):
    # Crea un MarkerArray
    marker_array = MarkerArray()
    
    # Configura propiedades comunes del Marker
    for i, center in enumerate(centers):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "borders"
        marker.id = marker_id + i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = 0  # Asumiendo que es en el plano XY
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = radius * 2  # Diámetro en X
        marker.scale.y = radius * 2  # Diámetro en Y
        marker.scale.z = radius * 2  # Diámetro en Z
        if i==id:
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.5)
        else:
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.5)
            
        #marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)  # Color verde con algo de transparencia
        marker.lifetime = rospy.Duration()
        
        marker_array.markers.append(marker)
    
    return marker_array

def main():
    rospy.init_node('border_marker_publisher')
    pub = rospy.Publisher('border_markers', MarkerArray, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # Supongamos que estos son tus centros
    centers = [
        [-8.305, -13.209],
        [-7.090, -4.358]
    ]

    while not rospy.is_shutdown():
        markers = create_border_marker(centers, radius=0.5)
        pub.publish(markers)
        rate.sleep()

if __name__ == '__main__':
    main()

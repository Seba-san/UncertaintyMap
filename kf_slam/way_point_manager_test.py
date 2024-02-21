import rospy
from geometry_msgs.msg import PoseArray

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg  import Path

import time

if __name__=='__main__':
    rospy.init_node('way_point_manager_test_node')
    path_request =rospy.Publisher('/path_rrt_star',Path,queue_size=1)
    time.sleep(5)
    
    P=Path()
    P.header.seq=11
    p=PoseStamped()
    p.pose.position.x=0
    p.pose.position.y=0
    P.poses.append(p)
    del p
    p=PoseStamped()
    p.pose.position.x=10
    p.pose.position.y=0
    P.poses.append(p)
    p=PoseStamped()
    p.pose.position.x=0
    p.pose.position.y=0
    P.poses.append(p)

    print(P.poses)
    path_request.publish(P)
    rospy.signal_shutdown('nodo test rrt star finalizado.')
    
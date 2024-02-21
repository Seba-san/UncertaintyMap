#!/usr/bin/env python

"""_summary_
Codigo realizado por chat gepete
captura las poses del slam y la de gazebo real y las publica con un mensaje del tipo Path
"""
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

#class TransformListenerNode:
#    def __init__(self):
#        rospy.init_node('transform_listener_node')
#        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
#        self.tf_listener = tf.TransformListener()
#        self.path = Path()
#        self.path.header.frame_id = 'robot1_tf/odom_groundtruth'
#
#    def publish_path(self, event):
#        try:
#            now = rospy.Time(0)
#            self.tf_listener.waitForTransform('robot1_tf/odom_groundtruth', '/kf_slam', now, rospy.Duration(1.0))
#            (trans, rot) = self.tf_listener.lookupTransform('robot1_tf/odom_groundtruth', '/kf_slam', now)
#            pose = PoseStamped()
#            pose.header.stamp = now
#            pose.header.frame_id = 'robot1_tf/odom_groundtruth'
#            pose.pose.position.x = trans[0]
#            pose.pose.position.y = trans[1]
#            pose.pose.position.z = trans[2]
#            pose.pose.orientation.x = rot[0]
#            pose.pose.orientation.y = rot[1]
#            pose.pose.orientation.z = rot[2]
#            pose.pose.orientation.w = rot[3]
#            self.path.poses.append(pose)
#            self.path.header.stamp = rospy.Time.now()
#            self.path_pub.publish(self.path)
#        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#            rospy.logwarn('Failed to get transformation')
#
#    def run(self):
#        rospy.Timer(rospy.Duration(1), self.publish_path)
#        rospy.spin()
#
#if __name__ == '__main__':
#    try:
#        node = TransformListenerNode()
#        node.run()
#    except rospy.ROSInterruptException:
#        pass

class TransformListenerNode:
    def __init__(self):
        rospy.init_node('transform_listener_node')
        self.tf_listener = tf.TransformListener()
        self.tracked_paths = {}

    def track_transform(self, transform_name, transform_topic, path_topic, frame_id):
        path_pub = rospy.Publisher(path_topic, Path, queue_size=10)
        path = Path()
        path.header.frame_id = frame_id
        self.tracked_paths[transform_name] = (path_pub, path, transform_topic)

    def publish_paths(self, event):
        # now = rospy.Time(0)
        now=rospy.Time.now()
        for transform_name, (path_pub, path, transform_topic) in self.tracked_paths.items():
            try:
                self.tf_listener.waitForTransform(path.header.frame_id, transform_topic, now, rospy.Duration(1.0))
                (trans, rot) = self.tf_listener.lookupTransform(path.header.frame_id, transform_topic, now)
                pose = PoseStamped()
                pose.header.stamp = now
                pose.header.frame_id = path.header.frame_id
                pose.pose.position.x = trans[0]
                pose.pose.position.y = trans[1]
                pose.pose.position.z = trans[2]
                pose.pose.orientation.x = rot[0]
                pose.pose.orientation.y = rot[1]
                pose.pose.orientation.z = rot[2]
                pose.pose.orientation.w = rot[3]
                path.poses.append(pose)
                path.header.stamp = rospy.Time.now()
                path_pub.publish(path)
            #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #    rospy.logwarn(f'Failed to get transformation for {transform_name}')
            except:
                rospy.logwarn(f'Failed to get transformation for {transform_name}')


    def run(self):
        rospy.Timer(rospy.Duration(1), self.publish_paths)
        rospy.spin()

if __name__ == '__main__':
    try:
        node = TransformListenerNode()
        node.track_transform("slam", "kf_slam", "/path_slam", 'robot1_tf/odom_groundtruth')
        node.track_transform("chassis", "robot1_tf/chassis", "/path_chassis", 'robot1_tf/odom_groundtruth')
        node.run()
    except rospy.ROSInterruptException:
        pass
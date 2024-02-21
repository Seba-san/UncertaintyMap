#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from pynput import keyboard
import sys

def on_press(key):
    global pub
    msg = Point()
    
    if key == keyboard.Key.up:
        msg.y = 10
    elif key == keyboard.Key.down:
        msg.y = -10
    elif key == keyboard.Key.left:
        msg.x = -10
    elif key == keyboard.Key.right:
        msg.x = 10
    elif key == keyboard.KeyCode.from_char('5'):
        msg.x = 0
        msg.y = 0
    elif key==keyboard.KeyCode.from_char('q'):
        #raise 0
        #rospy.signal_shutdown("Exiting")
        sys.exit(0)
    
    pub.publish(msg)

def start_node():
    rospy.init_node('keyboard_control_node', anonymous=True)
    global pub
    pub = rospy.Publisher('/dot_command', Point, queue_size=1)
    print('Iniciando nodo de control por teclado')
    with keyboard.Listener(on_press=on_press) as listener:
        try:
            listener.join()
        except KeyboardInterrupt:
            pass
        #listener.join()

if __name__ == '__main__':
    
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass

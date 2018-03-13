#!/usr/bin/env python
import math
import numpy as np
import rospy
from pynput import keyboard

from rosflight_msgs.msg import Command
del_a = 0.0
del_e = 0.02967
del_r = 0.0
del_f = 0.0
cmd = Command()
pub = rospy.Publisher('/fixedwing/command', Command, queue_size = 10)
rospy.init_node('key_contr', anonymous=True)
rate = rospy.Rate(60)

def on_press(key):
    global del_a, del_e, del_r, del_f, cmd
    if format(key.char) == 'w':
        del_e += -0.05
    elif format(key.char) == 's':
        del_e += 0.05
    elif format(key.char) == 'a':
        del_a += -0.05
    elif format(key.char) == 'd':
        del_a += 0.05
    elif format(key.char) == 'q':
        del_r += -0.05
    elif format(key.char) == 'e':
        del_r += 0.05
    elif format(key.char) == 'j':
        del_f += -0.05
    elif format(key.char) == 'i':
        del_f += 0.05
    if del_a >= 0.6 or del_a <= -0.6:
        del_a = 0.6 * np.sign(del_a)
    else:
        del_a = del_a
    if del_e >= 0.6 or del_e <= -0.6:
        del_e = 0.6 * np.sign(del_e)
    else:
        del_e = del_e
    if del_r >= 0.6 or del_r <= -0.6:
        del_r = 0.6 * np.sign(del_r)
    else:
        del_r = del_r
    if del_f >= 1.0:
        del_f = 1.0
    elif del_f <= 0.0:
        del_f = 0.0
    else:
        del_f = del_f
    cmd.x = del_a
    cmd.y = del_e
    cmd.z = del_r
    cmd.F = del_f
    pub.publish(cmd)
    rate.sleep()
        
def on_release(key):
    global del_a, del_e, del_r, del_f
    
    if format(key.char) == 'w':
        del_e = 0.02967
    elif format(key.char) == 's':
        del_e = 0.02967
    elif format(key.char) == 'a':
        del_a = 0.0
    elif format(key.char) == 'd':
        del_a = 0.0
    elif format(key.char) == 'q':
        del_r = 0.0
    elif format(key.char) == 'e':
        del_r = 0.0
    cmd.x = del_a
    cmd.y = del_e
    cmd.z = del_r
    cmd.F = del_f
    pub.publish(cmd)
    rate.sleep()

# Collect events until released
with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()

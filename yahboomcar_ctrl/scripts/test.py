import os
import time
import rospy
import threading
from time import sleep
from sensor_msgs.msg import Joy
from yahboomcar_msgs.msg import *
from yahboomcar_msgs.srv import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool
from actionlib_msgs.msg import GoalID

print(joy_data.axes[0])

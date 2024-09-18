import unittest
import rospy
from hector_moveit_exploration.msg import MoveAction

class TestDroneFunctionality(unittest.TestCase):

    def __init__(self):
        self.pub = None
        self.x = None
        self.y = None
        self.z = None

    def setUp(self):
        print('setting up for testing drone functionality')
        rospy.init_node('test_drone', anonymous=True, disable_signals=True)
        self.pub = rospy.Publisher('/drone/do_action', MoveAction, queue_size=10)
        rospy.sleep(1)

    def pose_callback(self, data):
        position = data.pose.position
        self.x = position.x
        self.y = position.y
        self.z = position.z

    def test_move_to_1(self):
        rate = rospy.Rate(1)

        move_action = MoveAction()
        move_action.x = 1.0
        move_action.y = 2.0
        move_action.z = 3.0
        move_action.action = "X"

        rospy.loginfo("Publishing MoveAction for test # 1")
        self.pub.publish(move_action)
        rate.sleep()

        rospy.sleep(10)

        e_distance = (abs(move_action.x - self.x)**2 + abs(move_action.y - self.y)**2 + abs(move_action.z - self.z) ** 2)
        self.assertLess(e_distance, 1)


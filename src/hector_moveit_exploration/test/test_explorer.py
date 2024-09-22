import unittest
import rospy
from hector_moveit_exploration.msg import MoveAction
from nav_msgs.msg import Odometry

class TestDroneFunctionality(unittest.TestCase):


    def setUp(self):
        self.pub = None
        self.x = None
        self.y = None
        self.z = None

        print('setting up for testing drone functionality')
        rospy.init_node('test_drone', anonymous=True, disable_signals=True)
        self.pub = rospy.Publisher('/drone/do_action', MoveAction, queue_size=10)
        rospy.sleep(1)

        rospy.Subscriber('/ground_truth/state', Odometry, self.pose_callback)

        # Sleep to ensure subscriptions and ROS connections are set up
        rospy.sleep(1)

        # Wait for the first ground truth state message to be received
        try:
            rospy.wait_for_message('/ground_truth/state', Odometry, timeout=5)
        except rospy.ROSException:
            self.fail("Timeout while waiting for ground truth state message.")

    def pose_callback(self, data):
        position = data.pose.pose.position
        self.x = position.x
        self.y = position.y
        self.z = position.z

    def test_move_to_1(self):
        rate = rospy.Rate(1)

        # rospy.loginfo("Currently at pose %f, %f, %f", self.x, self.y, self.z);

        move_action = MoveAction()
        move_action.x = self.x + 2
        move_action.y = self.y + 2
        move_action.z = self.z + 2
        move_action.action = "move_to"

        # rospy.loginfo("Publishing MoveAction for test # 1")
        self.pub.publish(move_action)
        rate.sleep()

        rospy.sleep(15)

        e_distance = (abs(move_action.x - self.x)**2 + abs(move_action.y - self.y)**2 + abs(move_action.z - self.z) ** 2)
        self.assertLess(e_distance, 1)


if __name__ == "__main__":
    unittest.main()

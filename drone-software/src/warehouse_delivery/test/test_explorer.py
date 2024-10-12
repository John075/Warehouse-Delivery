import rospy
from nav_msgs.msg import Odometry
from warehouse_delivery.msg import MoveAction
import unittest

class TestDrone(unittest.TestCase):

    def setUp(self):
        self.pub = None
        self.x = None
        self.y = None
        self.z = None

        print('Setting up for testing drone functionality...')
        rospy.init_node('test_drone', anonymous=True, disable_signals=True)

        # Publisher to send move actions
        self.pub = rospy.Publisher('/drone/do_action', MoveAction, queue_size=10)
        rospy.sleep(1)
          
        # Subscribe to ground truth state to track position
        rospy.Subscriber('/ground_truth/state', Odometry, self.pose_callback)

        # Wait for the first ground truth state message to be received to initialize position
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
        # Wait until we have valid pose data
        self.assertIsNotNone(self.x, "No ground truth state received")
        
        move_action = MoveAction()
        move_action.x = self.x + 2
        move_action.y = self.y + 2
        move_action.z = self.z + 2
        move_action.action = "move_to"

        # Publish the move action
        self.pub.publish(move_action)

        # Check the movement over time
        rate = rospy.Rate(10)  
        timeout_time = rospy.get_time() + 10  # Timeout after 10 seconds
        goal_reached = False

        while rospy.get_time() < timeout_time:
            current_distance = ((move_action.x - self.x)**2 + (move_action.y - self.y)**2 + (move_action.z - self.z)**2) ** 0.5
            
            if current_distance < 2:
                goal_reached = True
                break
            rate.sleep()

        self.assertTrue(goal_reached, "The drone failed to reach the target position within the allowed time.")

if __name__ == '__main__':
    unittest.main()

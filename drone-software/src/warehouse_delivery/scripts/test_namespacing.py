#!/usr/bin/env python
import rospy
import rospkg
import roslaunch

def spawn_drones(num_drones):
    rospy.init_node('spawn_random_drones')

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('hector_moveit_gazebo')

    for i in range(num_drones):
        ns = "mg{}".format(i + 1)
        launch_file = "{}/launch/orchyard_navigation_ns.launch".format(package_path)

        launch_args = [
            "name:=quadrotor_{}".format(i + 1),
            "namespace2:={}".format(ns)
        ]

        # Create a ROSLaunchParent instance for each namespace
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_parent = roslaunch.parent.ROSLaunchParent(
            uuid, [(launch_file, launch_args)]
        )

        launch_parent.start()

    rospy.spin()


if __name__ == "__main__":
    num_drones = 1
    print("Spawning {} drones".format(num_drones));
    spawn_drones(num_drones)

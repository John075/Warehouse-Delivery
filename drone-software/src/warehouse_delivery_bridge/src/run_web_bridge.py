#!/usr/bin/env python

import rospy
import requests
import sseclient  # this will handle updates from backend for changes
import json
import threading
import sys

from warehouse_delivery.msg import MoveAction
from warehouse_delivery_bridge.msg import DeliveryStatus

base_url = "http://localhost:8080/api"


class DroneDataHandler:
    def __init__(self, drone_id, publisher):
        # Keep copies of our drone ID, data, current action and current package
        self.drone_id = drone_id
        self.drone_data = None
        self.action = None
        self.package = None

        # We use locks with concurrent data access to ensure we don't have race condition issues
        self.action_data_lock = threading.Lock()
        self.drone_data_lock = threading.Lock()

        # Initialize our action, publisher, callback subscriber, data accessor and updater thread
        self.publisher = publisher
        self.accessor_thread = threading.Thread(target=self._data_accessor)
        self.updater_thread = threading.Thread(target=self._data_updater)
        self.global_subscriber = rospy.Subscriber('/drone{}/status_callback'.format(self.drone_id), DeliveryStatus,
                                                  self.status_update_callback)

    def start(self):
        self.accessor_thread.start()
        self.updater_thread.start()

    def publish_action(self, x, y, z):
        move_action = MoveAction()
        move_action.x = x
        move_action.y = y
        move_action.z = z
        move_action.action = "move_to"

        # Publish the move action
        self.publisher.publish(move_action)

    def status_update_callback(self, msg):
        # Check if our action has completed.
        if self.action == "DELIVERING_PACKAGE" and msg.status == "COMPLETED":
            with self.action_data_lock:
                self.action = "RETURNING_TO_WAREHOUSE"

            # TODO: Either provide a height to go to in our database, or we can use OpenCV / some other framework to
            #  self determine
            with self.drone_data_lock:
                self.publish_action(self.drone_data.warehouse.latitude, self.drone_data.warehouse.longitude, 10)
        elif self.action == "RETURNING_TO_WAREHOUSE" and msg.status == "COMPLETED":
            with self.action_data_lock:
                self.action = None
            # Now we wait for our data accessor thread to

    def _data_accessor(self):
        while not rospy.is_shutdown():
            with self.drone_data_lock and self.action_data_lock:
                if self.action is None:
                    # Check if we have any new packages
                    options = []
                    if len(self.shared_data.packages) >= 1:
                        for package in self.shared_data.packages:
                            if package.status != 'DELIVERED':
                                options.append(package)

                        lowest_found_priority = sys.maxint
                        best_option = None
                        for option in options:
                            if option.priorty < lowest_found_priority:
                                lowest_found_priority = option.priorty
                                best_option = option

                        if best_option is not None:
                            self.action = "DELIVERING_PACKAGE"
                            self.package = best_option

            rospy.sleep(1)

    def _data_updater(self):
        print("Starting Updater Thread for drone ID {}".format(self.drone_id))

        url = base_url + "/drone/1/updates"
        response = requests.get(url, stream=True)
        client = sseclient.SSEClient(response)

        if response.status_code != 200:
            print("Failed to connect to web server... status code: {}".format(response.status_code))

        for event in client.events():
            if event.event != "message":
                try:
                    data = json.loads(event.data)
                    print("Received update: {}".format(event.event))
                    print("Received update: {}".format(data))
                except Exception as e:
                    print('Failed to decode JSON from server: {}'.format(e))
                    print('Unknown data: {}'.format(event.data))
                    continue

                if event.event == "drone-general" or event.event == "drone-initial":
                    with self.drone_data_lock:
                        self.shared_data = data

                    print('Updated drone structure to: {}'.format(data))
                else:
                    print('Unknown event: {}'.format(event))
            else:
                print('Received message', event.data)


if __name__ == "__main__":
    try:
        print('Setting up for listening to web server...')
        rospy.init_node('web_node', anonymous=True, disable_signals=True)

        pub = rospy.Publisher('/drone/do_action', MoveAction, queue_size=10)
        rospy.sleep(1)

        # TODO: Fetch all drones and creates instances for all
        drone_1_updater = DroneDataHandler(1, pub)
        drone_1_updater.start()
    except rospy.ROSInterruptException:
        pass

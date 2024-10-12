#!/usr/bin/env python

import rospy
import requests
import sseclient  # this will handle updates from backend for changes
from warehouse_delivery.msg import MoveAction
import json

base_url = "http://localhost:8080/api"
def listen_to_updates(pub):
    print("Starting ROS node to listen to updates")

    url = "http://localhost:8080/api/drone/1/updates"
    response = requests.get(url, stream=True)
    client = sseclient.SSEClient(response)

    if response.status_code != 200:
        print("Failed to connect to web server... status code: {}".format(response.status_code))

    instantiated_data = None
    # Listen to incoming events
    for event in client.events():
        if event.event == "drone-update":
            try:
                drone_data = json.loads(event.data)
                print("Received drone update:", drone_data)
            except json.JSONDecodeError as e:
                print('Failed to decode JSON from server:', e)
                continue

            prior_data = instantiated_data
            instantiated_data = drone_data

            print('successfully fetched drones from database')
            for drone in drone_data:
               drone_collection.append(drone)
               print("Received drone update:", event.data)
        else:
                print(event)


if __name__ == "__main__":
    try:
        print('Setting up for listening to web server...')
        rospy.init_node('web_node', anonymous=True, disable_signals=True)

        pub = rospy.Publisher('/drone/do_action', MoveAction, queue_size=10)
        rospy.sleep(1)

        listen_to_updates(pub)
    except rospy.ROSInterruptException:
        pass
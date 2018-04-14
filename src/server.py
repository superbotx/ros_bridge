#!/usr/bin/env python

import json
import yaml
import util
import time
from threading import Thread
import rospy
from flask import Flask, request
from flask_socketio import SocketIO, send, emit

streaming_app = Flask(__name__)
socketio = SocketIO(streaming_app)

ros_nodes = None
ros_nodes_hash = None
ros_topics = {}
ros_publishers = {}

@socketio.on('forward_ros_msg')
def handle_my_custom_event(topic_name, message_type, payload, count=1, rate=1):
    global ros_publishers
    print('received forward: ' + topic_name + ' ' + message_type + ' ' + str(payload))
    if topic_name not in ros_publishers:
        msg_module = util.get_ros_message_package(message_type)
        pub = rospy.Publisher(topic_name, msg_module, queue_size=10)
        ros_publishers[topic_name] = pub
    ros_msg = util.convert_dictionary_to_ros_message(message_type, payload)
    interval = rospy.Rate(float(rate))
    for i in range(count):
        ros_publishers[topic_name].publish(ros_msg)
        interval.sleep()

class ros_callback():
    def __init__(self, topic_name):
        self.topic_name = topic_name

    def watch(self, data):
        payload = util.convert_ros_message_to_dictionary(data)
        namespace = '/' + self.topic_name
        socketio.emit('ros_message', ("payload", payload), broadcast=True, namespace=namespace)

def launch_subscriber(topic_name, msg, watcher):
    rospy.Subscriber(topic_name, msg, watcher.watch)
    rospy.spin()

def refresh_ros():
    while True:
        print('Refreshing ros information ...')
        global ros_nodes, ros_nodes_hash, ros_topics, ros_publishers
        ros_nodes, ros_nodes_hash = util.get_ros_nodes()
        tmp_ros_topics = util.get_ros_topics()
        for tmp_ros_topic in tmp_ros_topics:
            if tmp_ros_topic['name'] not in ros_topics and tmp_ros_topic['name'] not in ros_publishers:
                print('adding '+tmp_ros_topic['name']+' to watch')
                ros_topics[tmp_ros_topic['name']] = tmp_ros_topic
                watcher = ros_callback(tmp_ros_topic['name'])
                sub_t = Thread(target=launch_subscriber, kwargs={'topic_name':tmp_ros_topic['name'],'msg':tmp_ros_topic['module'],'watcher':watcher})
                sub_t.start()
        time.sleep(5)

def run_streaming_server(app, host, port):
    print('Starting streaming server...')
    socketio.run(app, port=port, host=host)

if __name__ == "__main__":
    streaming_server_t = Thread(target=run_streaming_server, kwargs={'app':streaming_app,'host':'localhost','port':5000})
    streaming_server_t.start()
    rospy.init_node('bridge', anonymous=True)
    refresher_t = Thread(target=refresh_ros)
    refresher_t.start()

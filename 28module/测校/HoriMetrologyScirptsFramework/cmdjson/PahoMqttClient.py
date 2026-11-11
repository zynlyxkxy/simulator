# -*- coding: utf-8 -*-
import paho.mqtt.client as mqtt
import time
from queue import Queue
from collections import deque
import numpy as np
# import cv2
import json
import subprocess
import threading


# import StageFlsStablity_ui as ui

class PahoMqttClient:
    pl = ''

    def __init__(self):
        self.mqttClient = mqtt.Client()
        self.mqttQueue = Queue(-1)
        self.imgQueue = Queue(-1)
        self.lock = threading.RLock()

    def connect(self, host, port):
        self.mqttClient.connect(host, port, 60)
        self.mqttClient.loop_start()

    # publish 消息
    def publish(self, topic, payload, qos):
        self.mqttClient.publish(topic, payload, qos)

    # 消息处理函数
    def on_message_come(self, client, userdata, msg):
        self.pl = msg.payload.decode()
        self.mqttQueue.put(msg)

    def read_json(self, file_path):
        with open(file_path, 'r') as file:
            return json.load(file)

    def save_json(self, msg):
        with open("data.json", "r+") as location:
            locate = json.load(location)
            n = 0
            temp = json.loads(json.loads(json.loads(msg.payload.decode())['context'])['context'])['res']['data']
            locate.update(temp)
            with open("data.json", "r+") as location2:
                json.dump(temp, location2, indent=4)
        print('Save Success:\n')
        print(msg.payload.decode())

    def decode_json(self, msg):
        print('msg ', msg)
        data_dict = json.loads(msg)
        content1 = data_dict['context']
        # data_dict = json.loads(content1)
        # content2 = data_dict['context']
        data_dict = json.loads(content1)
        content3 = data_dict['res']
        counts = content3['count']
        data = content3['data']
        datamean = np.mean(np.array(data))
        return datamean

    # subscribe 消息
    def subscribe(self, topics):
        # mqttClient.subscribe("plcMsg", 1)
        subscribe_topics = []
        for topic in topics:
            subscribe_topics.append([topic, 2])
        self.mqttClient.subscribe(subscribe_topics)
        time.sleep(0.1)
        print(f"subscribe {topics}")
        self.mqttClient.on_message = self.on_message_come  # 消息到来处理函数

    # unsubscribe 消息
    def unsubscribe(self, topics):
        unsubscribe_topics = []
        for topic in topics:
            unsubscribe_topics.append(topic)
        self.mqttClient.unsubscribe(topics)
        time.sleep(0.3)
        print(f"unsubscribe {topics}")

    def send_cmd(self, payload, topic="VisionRcp", qos=2):
        self.publish(topic, payload, qos)

    def rcv_res(self):
        msg = self.mqttQueue.get()
        return msg

    def async_rcv_res(self):
        if self.mqttQueue.empty():
            return ' '
        print('async_rcv_res enter')
        self.lock.acquire()
        msg = self.mqttQueue.get()

        data = msg.payload.decode()
        self.lock.release()

        return data

    def rcv_img(self):
        return self.imgQueue.get()

    def close(self):
        self.mqttClient.disconnect()

    def get_connect_statue(self):
        return self.mqttClient.is_connected()

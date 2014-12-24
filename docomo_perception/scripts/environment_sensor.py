#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from docomo_perception.srv import (EnvironmentSensorService, EnvironmentSensorServiceResponse)
from docomo_perception.msg import EnvironmentSensor

import json
import requests
import yaml
import copy
import threading

global APIHOST, APIKEY, LONGTITUDE, LATITUDE, RANGE
setting_path = "/var/lib/robot/docomo_environment_sensor_settings.yaml"


class DocomoEnvironmentSensoeNode(object):
    def __init__(self):
        self.srv = rospy.Service("environment_sensor", EnvironmentSensorService, self.serviceCallback)
        self.lock = threading.Lock()

    def serviceCallback(self, req):
        params = {
            "APIKEY": APIKEY,
            "with_data": "true",
            "order": "prefecture,asc,city,asc"
            }
        if req.base_id and len(req.base_id) > 0 and req.base_id[0] != "":
            params["id"] = req.base_id
        else:
            if req.latitude:
                params["lat"] = str(req.latitude)
            else:
                params["lat"] = LATITUDE
            if req.longtitude:
                params["lon"] = str(req.longtitude)
            else:
                params["lon"] = LONGTITUDE
            if req.range:
                params["range"] = str(req.range)
            elif RANGE:
                params["range"] = RANGE

        envlist = []
        for data_type in ["1013", "1213", "2221"]:
            p = copy.deepcopy(params)
            p["data_type"] = data_type
            self.lock.acquire()
            rospy.loginfo("param: %s", p)
            urlres = requests.get(APIHOST, params=p)
            self.lock.release()
            rospy.loginfo("sent request to %s -> %d", urlres.url, urlres.status_code)
            rospy.loginfo("content: %s", urlres.content)
            if urlres.status_code is not requests.codes.ok:
                rospy.logerr("bad status code: %d", urlres.status_code)
                return EnvironmentSensorServiceResponse()
            resdic = json.loads(urlres.content)

            for s in resdic["sensor"]:
                try:
                    idx = [e.base_id for e in envlist].index(s["id"])
                    env_data = s["environment_data"][0]
                    dt = int(env_data["data_type"])
                    val = float(env_data["val"][0])
                    rospy.logwarn("val2: %f", val)
                    if dt == 1013:
                        envlist[idx].temperature = val
                    elif dt == 1213:
                        envlist[idx].precipitation = val
                    elif dt == 2221:
                        envlist[idx].uv_index = val
                    else:
                        rospy.logwarn("received invalid data_type: %s", env_data["data_type"])
                except:
                    sensor_msg = EnvironmentSensor()
                    sensor_msg.base_id = s["id"]
                    sensor_msg.name = s["name"]
                    sensor_msg.prefecture = s["prefecture"]
                    sensor_msg.city = s["city"]
                    sensor_msg.latitude = float(s["lat"])
                    sensor_msg.longtitude = float(s["lon"])
                    env_data = s["environment_data"][0]
                    dt = int(env_data["data_type"])
                    val = float(env_data["val"][0])
                    if dt == 1013:
                        sensor_msg.temperature = val
                    elif dt == 1213:
                        sensor_msg.precipitation = val
                    elif dt == 2221:
                        sensor_msg.uv_index = val
                    else:
                        rospy.logwarn("received invalid data_type: %s", env_data["data_type"])
                    envlist.append(sensor_msg)
        res = EnvironmentSensorServiceResponse()
        res.data = envlist
        return res

def load_environment_sensor_settings():
    global APIHOST, APIKEY, LONGTITUDE, LATITUDE, RANGE

    try:
        with open(setting_path) as f:
            key = yaml.load(f)
            APIHOST = key['APIHOST']
            APIKEY = key['APIKEY']
            if key.has_key("LONGTITUDE"):
                LONGTITUDE = key["LONGTITUDE"]
            if key.has_key("LATITUDE"):
                LATITUDE = key["LATITUDE"]
            if key.has_key("RANGE"):
                RANGE = key["RANGE"]
            rospy.loginfo("loaded settings")
    except IOError as e:
        rospy.logerr('"%s" not found : %s' % (settings_path, e))
        exit(-1)
    except Exception as e:
        rospy.logerr("failed to load settings: %s", e)
        rospy.logerr("check if exists valid setting file in %s", settings_path)
        exit(-1)


if __name__ == '__main__':
    rospy.init_node("docomo_environment_sensor_node")
    n = DocomoEnvironmentSensoeNode()
    load_environment_sensor_settings()
    rospy.spin()

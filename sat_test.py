import asyncio
import unittest
from unittest import TestCase, IsolatedAsyncioTestCase
from time import sleep
import sys
import os
import pdb

from sat_utils import absolute_time_converter_utc_manual, absolute_time_converter_utc_string
import sat_utils as sat_utils

import json
import time


# opening a file that contains a sample shared storage
f = open("test_orbit_config.json")
shared_storage = json.load(f)


    
orbit_1 = shared_storage["swarm"]["cubesat_1"]["orbit"]
orbit_2 = shared_storage["swarm"]["cubesat_2"]["orbit"]

attitude_provider_1 = {"type":"moving_body_tracking", "parameters": shared_storage["swarm"][shared_storage["swarm"]["cubesat_1"]["orbit"]["attitude"]]["orbit"]}
attitude_provider_2 = {"type":"moving_body_tracking", "parameters": shared_storage["swarm"][shared_storage["swarm"]["cubesat_2"]["orbit"]["attitude"]]["orbit"]}

data = {
    "sender_ID": "cubesat_2",
    "time_sent": "2021-12-05T00:"+str(10)+":00.000",
    "data":{
        "time": "2021-12-05T00:"+str(12)+":00.000"
    }
}


time = absolute_time_converter_utc_string(data["data"]["time"])

# Creating propagators and attitude provider from orbit configuration of satellites
propagator_1 = sat_utils.analytical_propagator(orbit_1)
propagator_2 = sat_utils.analytical_propagator(orbit_2)
attitude_provider_1 = sat_utils.attitude_provider_constructor(attitude_provider_1["type"], attitude_provider_1["parameters"])
attitude_provider_2 = sat_utils.attitude_provider_constructor(attitude_provider_2["type"], attitude_provider_2["parameters"])

# Setting attitude and propagating the orbit 
propagator_1.setAttitudeProvider(attitude_provider_1)
propagator_2.setAttitudeProvider(attitude_provider_2)
state_1 = propagator_1.propagate(time)
state_2 = propagator_2.propagate(time)

# Updating param
cubesat_1_param = sat_utils.get_keplerian_parameters(state_1)
cubesat_1_param.update({"attitude": "cubesat_2"})
cubesat_1_param.update({"frame": "EME"})
cubesat_2_param = sat_utils.get_keplerian_parameters(state_2)
cubesat_2_param.update({"attitude": "cubesat_1"})
cubesat_2_param.update({"frame": "EME"})

# Checking to see if simulation_timestep_propagate updated params correctly
# self.assertTrue(shared_storage["swarm"]["cubesat_1"]["orbit"] == cubesat_1_param)
# self.assertTrue(shared_storage["swarm"]["cubesat_2"]["orbit"] == cubesat_2_param)
# self.assertTrue(shared_storage["swarm"]["cubesat_1"]["orbit"]["attitude"] == "cubesat_2")
# self.assertTrue(shared_storage["swarm"]["cubesat_2"]["orbit"]["attitude"] == "cubesat_1")
print(state_1)
print(state_1.attitude)
# print(shared_storage["swarm"]["cubesat_2"]["target_in_view"])
# # Making sure that phonebook gets updated properly
# if i <= 48:
#     self.assertTrue(shared_storage["sat_phonebook"]["cubesat_2"])
#     self.assertTrue(shared_storage["swarm"]["cubesat_2"]["target_in_view"])
# else:
#     self.assertFalse(shared_storage["sat_phonebook"]["cubesat_2"])
#     self.assertFalse(shared_storage["swarm"]["cubesat_2"]["target_in_view"])


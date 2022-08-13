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
import agile_sat_utils
import env_orekit


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



year, month, day, hr, minute, sec = 2018, 8, 1, 9, 30, 00.00
date = [year, month, day, hr, minute, sec]

dry_mass = 500.0
fuel_mass = 150.0
mass = [dry_mass, fuel_mass]
duration = 24.0 * 60.0 ** 2 * 2

# set the sc initial state
a = 10000.0e3  # semi major axis (m) (altitude)
e = 0.1  # eccentricity
i = 5.0  # inclination
omega = 10.0  # perigee argument
raan = 10.0  # right ascension of ascending node
lM = 10.0  # mean anomaly
state = [a, e, i, omega, raan, lM]

# target state
a_targ = 35000.0e3  # altitude
e_targ = 0.3
i_targ = 10.0
omega_targ = 10.0
raan_targ = 10.0
lM_targ = 10.0
state_targ = [a_targ, e_targ, i_targ, omega_targ, raan_targ, lM_targ]
stepT = 500.0


env = env_orekit.OrekitEnv(state, state_targ, date, duration, mass, stepT)
n_actions = env.r_initial_state
print(n_actions)


# Checking to see if simulation_timestep_propagate updated params correctly
# self.assertTrue(shared_storage["swarm"]["cubesat_1"]["orbit"] == cubesat_1_param)
# self.assertTrue(shared_storage["swarm"]["cubesat_2"]["orbit"] == cubesat_2_param)
# self.assertTrue(shared_storage["swarm"]["cubesat_1"]["orbit"]["attitude"] == "cubesat_2")
# self.assertTrue(shared_storage["swarm"]["cubesat_2"]["orbit"]["attitude"] == "cubesat_1")
# print(state_1)
# print(state_1.attitude)
# print(shared_storage["swarm"]["cubesat_2"]["target_in_view"])
# # Making sure that phonebook gets updated properly
# if i <= 48:
#     self.assertTrue(shared_storage["sat_phonebook"]["cubesat_2"])
#     self.assertTrue(shared_storage["swarm"]["cubesat_2"]["target_in_view"])
# else:
#     self.assertFalse(shared_storage["sat_phonebook"]["cubesat_2"])
#     self.assertFalse(shared_storage["swarm"]["cubesat_2"]["target_in_view"])


import asyncio
import unittest
from unittest import TestCase, IsolatedAsyncioTestCase
from time import sleep
import sys
import os
import pdb
import math

from sat_utils import absolute_time_converter_utc_manual, absolute_time_converter_utc_string
import sat_utils as sat_utils

import json
import time


"""
attitude_provider_constructor test
"""
moving_body = "moving_body_tracking"
moving_body_param = {
            "eccentricity": 0.0008641,
            "semimajor_axis": 6801395.04,
            "inclination": 87.0,
            "perigee_argument": 10.0,
            "right_ascension_of_ascending_node": 10.0,
            "anomaly": 0.0,
            "anomaly_type": "TRUE",
            "orbit_update_date":"2021-12-02T00:00:00.000",
            "frame": "EME"}

orbit_params = {
            "eccentricity": 0.0008641,
            "semimajor_axis": 6801395.04,
            "inclination": 87.0,
            "perigee_argument": 20.0,
            "right_ascension_of_ascending_node": 10.0,
            "anomaly": 0.0,
            "anomaly_type": "TRUE",
            "orbit_update_date":'2021-12-02T00:00:00.000',
            "frame": "EME"}

orbit_propagator_1 = sat_utils.analytical_propagator(orbit_params)
orbit_propagator_2 = sat_utils.analytical_propagator(orbit_params)

attitude_m_body = sat_utils.attitude_provider_constructor(moving_body, moving_body_param)
attitude_m_body_1 = sat_utils.moving_body_pointing_law(sat_utils.analytical_propagator(moving_body_param),moving_body_param)

orbit_propagator_1.setAttitudeProvider(attitude_m_body)
orbit_propagator_2.setAttitudeProvider(attitude_m_body_1)

time_to_propagate = sat_utils.absolute_time_converter_utc_string('2022-05-02T00:00:00.000')

state_1 = orbit_propagator_1.propagate(time_to_propagate)
state_2 = orbit_propagator_2.propagate(time_to_propagate)

print(state_1.getAttitude().getSpin())

    # self.assertTrue((state_1.getAttitude().getSpin().toString() == state_2.getAttitude().getSpin().toString()))

#     ground = "ground_tracking"
#     ground_param = {
#         "latitude": 12.0,
#         "altitude": 2343.0,
#         "longitude":12.0
#         }
#     parameters = {
#                 "eccentricity": 0.0008641,
#                 "semimajor_axis": 6801395.04,
#                 "inclination": 87.0,
#                 "perigee_argument": 10.0,
#                 "right_ascension_of_ascending_node": 10.0,
#                 "anomaly": math.radians(0.0),
#                 "anomaly_type": "TRUE",
#                 "orbit_update_date":'2021-12-02T00:00:00.000',
#                 "frame": "EME"}

#     orbit_propagator_1 = sat_utils.analytical_propagator(parameters)
#     orbit_propagator_2 = sat_utils.analytical_propagator(parameters)
#     ground_param["frame"] = sat_utils.frame_to_string(orbit_propagator_1.getFrame())

#     attitude_provider_1 = sat_utils.attitude_provider_constructor(ground, ground_param)
#     attitude_provider_2 = sat_utils.ground_pointing_law(ground_param)

#     orbit_propagator_1.setAttitudeProvider(attitude_provider_1)
#     orbit_propagator_2.setAttitudeProvider(attitude_provider_2)

#     time_to_propagate = sat_utils.absolute_time_converter_utc_string('2022-05-02T00:00:00.000')
#     state_1 = orbit_propagator_1.propagate(time_to_propagate)
#     state_2 = orbit_propagator_2.propagate(time_to_propagate)

#     self.assertTrue((state_1.getAttitude().getSpin().toString() == state_2.getAttitude().getSpin().toString()))

# if __name__ == '__main__':
import orekit
from math import radians, degrees, sqrt, pi
import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import os, random

orekit.initVM()

from org.orekit.frames import FramesFactory, Frame
from org.orekit.bodies import OneAxisEllipsoid
from org.orekit.orbits import KeplerianOrbit, Orbit
from org.orekit.propagation import SpacecraftState
from org.orekit.propagation.analytical import KeplerianPropagator
from org.orekit.time import AbsoluteDate, TimeScalesFactory
from org.hipparchus.ode.nonstiff import DormandPrince853Integrator
from org.hipparchus.geometry.euclidean.threed import Vector3D
from org.orekit.propagation.numerical import NumericalPropagator
from org.orekit.propagation.sampling import OrekitFixedStepHandler
from org.orekit.orbits import OrbitType, PositionAngle
from org.orekit.forces.gravity.potential import GravityFieldFactory
from org.orekit.forces.gravity import HolmesFeatherstoneAttractionModel
from org.orekit.utils import IERSConventions, Constants
from org.orekit.forces.maneuvers import ConstantThrustManeuver
from org.orekit.frames import LOFType
from org.orekit.attitudes import LofOffset
from org.orekit.python import PythonEventHandler, PythonOrekitFixedStepHandler
from org.orekit.bodies import OneAxisEllipsoid, GeodeticPoint, CelestialBodyFactory
from org.orekit.attitudes import CelestialBodyPointed, TargetPointing, NadirPointing
from orekit.pyhelpers import setup_orekit_curdir
from org.orekit.forces.gravity import NewtonianAttraction
from org.orekit.utils import Constants
import sat_utils

from org.hipparchus.geometry.euclidean.threed import Vector3D
from java.util import Arrays
from orekit import JArray_double

setup_orekit_curdir()

FUEL_MASS = "Fuel Mass"

UTC = TimeScalesFactory.getUTC()
inertial_frame = FramesFactory.getEME2000()
attitude = LofOffset(inertial_frame, LOFType.LVLH)
EARTH_RADIUS = Constants.WGS84_EARTH_EQUATORIAL_RADIUS
MU = Constants.WGS84_EARTH_MU

class OrekitEnv:
    """
    This class uses Orekit to create an environment to propagate a satellite
    """

    def __init__(self, state, state_targ, date, duration, mass, stepT):
        """
        initializes the orekit VM and included libraries
        Params:
        _prop: The propagation object
        _initial_date: The initial start date of the propagation
        _orbit: The orbit type (Keplerian, Circular, etc...)
        _currentDate: The current date during propagation
        _currentOrbit: The current orbit paramenters
        _px: spacecraft position in the x-direction
        _py: spacecraft position in the y-direction
        _sc_fuel: The spacecraft with fuel accounted for
        _extrap_Date: changing date during propagation state
        _sc_state: The spacecraft without fuel
        """

        self._prop = None
        self._initial_date = None
        self._orbit = None
        self._currentDate = None
        self._currentOrbit = None

        self.dry_mass = mass[0]
        self.fuel_mass = mass[1]
        self.cuf_fuel_mass = self.fuel_mass
        self.initial_mass = self.dry_mass + self.fuel_mass


        self.px = []
        self.py = []
        self.pz = []
        self.a_orbit = []
        self.ex_orbit = []
        self.ey_orbit = []
        self.hx_orbit = []
        self.hy_orbit = []
        self.lv_orbit = []

        self.adot_orbit = []
        self.exdot_orbit = []
        self.eydot_orbit = []
        self.hxdot_orbit = []
        self.hydot_orbit = []

        self._sc_fuel = None
        self._extrap_Date = None
        self._targetOrbit = None
        self.target_px = []
        self.target_py = []
        self.target_pz = []

        self._orbit_tolerance = {'a': 10000, 'ex': 0.01, 'ey': 0.01, 'hx': 0.001, 'hy': 0.001, 'lv': 0.01}

        self.randomize = False
        self._orbit_randomizer = {'a': 4000.0e3, 'e': 0.2, 'i': 2.0, 'w': 10.0, 'omega': 10.0, 'lv': 5.0}
        self.seed_state = state
        self.seed_target = state_targ
        self.target_hit = False

        self.set_date(date)
        self._extrap_Date = self._initial_date
        self.create_orbit(state, self._initial_date, target=False)
        self.set_spacecraft(self.initial_mass, self.cuf_fuel_mass)
        self.create_Propagator()
        self.setForceModel()
        self.final_date = self._initial_date.shiftedBy(duration)
        self.create_orbit(state_targ, self.final_date, target=True)

        self.stepT = stepT
        self.action_space = 3  # output thrust directions
        self.observation_space = 10  # states
        self.action_bound = 0.6  # Max thrust limit
        self._isp = 3100.0

        self.r_target_state = np.array(
            [self._targetOrbit.getA(), self._targetOrbit.getEquinoctialEx(), self._targetOrbit.getEquinoctialEy(),
             self._targetOrbit.getHx(), self._targetOrbit.getHy(), self._targetOrbit.getLv()])

        self.r_initial_state = np.array([self._orbit.getA(), self._orbit.getEquinoctialEx(), self._orbit.getEquinoctialEy(),
                                  self._orbit.getHx(), self._orbit.getHy(), self._orbit.getLv()])

        self.r_target_state = self.get_state(self._targetOrbit)
        self.r_initial_state = self.get_state(self._orbit)

    def create_orbit(self, state, date, target=False):
        """
         Crate the initial orbit using Keplarian elements
        :param state: a state list [a, e, i, omega, raan, lM]
        :param date: A date given as an orekit absolute date object
        :param target: a target orbit list [a, e, i, omega, raan, lM]
        :return:
        """
        a, e, i, omega, raan, lM = state

        a += EARTH_RADIUS
        i = radians(i)
        omega = radians(omega)
        raan = radians(raan)
        lM = radians(lM)

        aDot, eDot, iDot, paDot, rannDot, anomalyDot = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        # Set inertial frame
        set_orbit = KeplerianOrbit(a, e, i, omega, raan, lM,
                                   aDot, eDot, iDot, paDot, rannDot, anomalyDot,
                                   PositionAngle.TRUE, inertial_frame, date, MU)

        if target:
            self._targetOrbit = set_orbit
        else:
            self._currentOrbit = set_orbit
            self._orbit = set_orbit

    def convert_to_keplerian(self, orbit):

        ko = KeplerianOrbit(orbit)
        return ko

    def set_spacecraft(self, mass, fuel_mass):
        """
        Add the fuel mass to the spacecraft
        :param mass: dry mass of spacecraft (kg, flaot)
        :param fuel_mass:
        :return:
        """
        sc_state = SpacecraftState(self._orbit, mass)
        # print(f'{sc_state.getMass()}')
        self._sc_fuel = sc_state.addAdditionalState (FUEL_MASS, fuel_mass)

    def create_Propagator(self, prop_master_mode=False):
        """
        Creates and initializes the propagator
        :param prop_master_mode: Set propagator to slave of master mode (slave default)
        :return:
        """
        # tol = NumericalPropagator.tolerances(1.0, self._orbit, self._orbit.getType())
        minStep = 0.001
        maxStep = 500.0

        position_tolerance = 60.0
        tolerances = NumericalPropagator.tolerances(position_tolerance, self._orbit, self._orbit.getType())
        abs_tolerance = JArray_double.cast_(tolerances[0])
        rel_telerance = JArray_double.cast_(tolerances[1])

        # integrator = DormandPrince853Integrator(minStep, maxStep, 1e-5, 1e-10)
        integrator = DormandPrince853Integrator(minStep, maxStep, abs_tolerance, rel_telerance)

        integrator.setInitialStepSize(10.0)

        numProp = NumericalPropagator(integrator)
        numProp.setInitialState(self._sc_fuel)
        numProp.setMu(MU)
        numProp.setOrbitType(OrbitType.KEPLERIAN)

        if prop_master_mode:
            output_step = 5.0
            handler = OutputHandler()
            numProp.setMasterMode(output_step, handler)
        else:
            numProp.setSlaveMode()

        self._prop = numProp
        self._prop.setAttitudeProvider(attitude)

    def ground_pointing_law(parameters):
        """
        Given a longitude, lattitude, and altitude it returns a ground pointing attitude law
        Args:
            parameters: dictionary containing at least...
                        parameters["frame"] = (str) "EME2000", "J2000", or "TEME" only
                        parameters["latitude"] = (float -- radians) latitude
                        parameters["longitude"] = (float -- radians) longitude
                        parameters["altitude"] = (float -- meters) altitude above sea level
        Returns:
            AttidueProvider: attitude law that tells the satellite to point at a specific point on the ground
        """
        ITRF = FramesFactory.getITRF(IERSConventions.IERS_2010, True)
        earth = OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                Constants.WGS84_EARTH_FLATTENING,
                                ITRF)
        point = GeodeticPoint(float(parameters["latitude"]), float(parameters["longitude"]), float(parameters["altitude"]))
        frame = sat_utils.string_to_frame(parameters["frame"])
        return TargetPointing(frame, point, earth)

    def reset(self):
        """
        Resets the orekit enviornment
        :return:
        """

        self._prop = None
        self._currentDate = None
        self._currentOrbit = None

        # Randomizes the initial orbit
        if self.randomize:
            self._orbit = None
            a_rand = self.seed_state[0]
            e_rand = self.seed_state[1]
            w_rand = self.seed_state[3]
            omega_rand = self.seed_state[4]
            lv_rand = self.seed_state[5]

            a_rand = random.uniform(self.seed_state[0]-self._orbit_randomizer['a'], self._orbit_randomizer['a']+ self.seed_state[0])
            e_rand = random.uniform(self.seed_state[1]-self._orbit_randomizer['e'], self._orbit_randomizer['e']+ self.seed_state[1])
            i_rand = random.uniform(self.seed_state[2]-self._orbit_randomizer['i'], self._orbit_randomizer['i']+ self.seed_state[2])
            w_rand = random.uniform(self.seed_state[3]-self._orbit_randomizer['w'], self._orbit_randomizer['w']+ self.seed_state[3])
            omega_rand = random.uniform(self.seed_state[4]-self._orbit_randomizer['omega'], self._orbit_randomizer['omega']+ self.seed_state[4])
            lv_rand = random.uniform(self.seed_state[5]-self._orbit_randomizer['lv'], self._orbit_randomizer['lv']+ self.seed_state[5])
            state = [a_rand, e_rand, i_rand, w_rand, omega_rand, lv_rand]
            self.create_orbit(state, self._initial_date, target=False)
        else:
            self._currentOrbit = self._orbit

        self._currentDate = self._initial_date
        self._extrap_Date = self._initial_date

        self.set_spacecraft(self.initial_mass, self.fuel_mass)
        self.cuf_fuel_mass = self.fuel_mass
        self.create_Propagator()
        self.setForceModel()

        self.px = []
        self.py = []
        self.pz = []

        self.a_orbit = []
        self.ex_orbit = []
        self.ey_orbit = []
        self.hx_orbit = []
        self.hy_orbit = []
        self.lv_orbit = []

        self.adot_orbit = []
        self.exdot_orbit = []
        self.eydot_orbit = []
        self.hxdot_orbit = []
        self.hydot_orbit = []

        state = np.array([self._orbit.getA() / self.r_target_state[0],
                          self._orbit.getEquinoctialEx(),
                          self._orbit.getEquinoctialEy(),
                          self._orbit.getHx(),
                          self._orbit.getHy(), 0, 0, 0, 0, 0])
        return state

class OutputHandler(PythonOrekitFixedStepHandler):
    """
    Implements a custom handler for every value
    """
    def init(self, s0, t):
        """
        Initilization at every prpagation step
        :param s0: initial state (spacecraft State)
        :param t: initial time (int)
        :return:
        """
        print('Orbital Elements:')

    def handleStep(self, currentState, isLast):
        """
        Perform a step at every propagation step
        :param currentState: current spacecraft state (Spacecraft state)
        :param isLast: last step in the propagation (bool)
        :return:
        """
        o = OrbitType.KEPLERIAN.convertType(currentState.getOrbit())
        print(o.getDate())
        print('a:{:5.3f}, e:{:5.3f}, i:{:5.3f}, theta:{:5.3f}'.format(o.getA(), o.getE(),
                                                                      degrees(o.getI()), degrees(o.getLv())))
        if isLast:
            print('this was the last step ')
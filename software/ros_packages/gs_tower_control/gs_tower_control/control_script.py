from ast import match_case
from copy import deepcopy

from curses import flash
from faulthandler import is_enabled
import math
import time
import typing
from enum import Enum
from typing import Callable, Sequence, TypeVar, Generic
from odrive.enums import ControlMode, InputMode
from odrive.enums import AxisState
from scipy.spatial.transform import Rotation
from gs_tower_control.coordinate_math import *
from gs_tower_interfaces.msg import AntennaControlStatus, AntennaControlManualInput
from gs_tower_interfaces.srv import AntennaControlService
from odrive_can.msg import ODriveStatus, ControlMessage, ControllerStatus
import odrive_can.srv
import rclpy.client
import rclpy.executors
import rclpy.logging
import rclpy.publisher
import rclpy.subscription
import rclpy.timer
import rclpy.node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

# ---------- HELPER CLASSES -----------

class AxisRange:

    _min: float
    _max: float

    def __init__(self, min: float, max: float) -> None:
        self.set(min, max)

    def get_min(self):
        return self._min
    
    def get_max(self):
        return self._max
    
    def set_min(self, min: float):
        if (min > self._max):
            raise ValueError("min must be less than or equal to max")
        
        self._min = min

    def set_max(self, max: float):
        if (max < self._min):
            raise ValueError("max must be greater than or equal to min")
        
        self._max = max

    def set(self, min, max):
        if (max < min):
            raise(ValueError("max must be greater than or equal to min"))
        
        self._min = min
        self._max = max

    def in_range(self, a: float) -> bool:
        return (a <= self._max) and (a >= self._min)
    
    def clamp(self, a: float) -> float:
        if (a > self._max):
            return self._max
        elif (a < self._min):
            return self._min
        else:
            return a
        

T = TypeVar('T')
class TemporalValue(Generic[T]):

    _val: typing.Optional[T]
    _time: float
    _maxTimeoutSec: float

    def __init__(self, value: typing.Optional[T] = None, maxTimeoutSec: float = math.inf):
        self.set_max_timeout_sec(maxTimeoutSec)
        self.update(value)

    def update(self, value: typing.Optional[T]):
        self._val = value
        self._time = time.time()

    def get_value(self) -> typing.Optional[T]:
        return deepcopy(self._val) #is a copy really necessary?
    
    def max_timeout_exceeded(self):
        return (time.time() - self._time) > self._maxTimeoutSec
    
    def get_timeout_sec(self) -> float:
        return (time.time() - self._time)
    
    def get_update_time(self) -> float:
        return self._time

    def get_max_timeout_sec(self) -> float:
        return self._maxTimeoutSec
    
    def set_max_timeout_sec(self, timeout: float):
        if (timeout < 0):
            raise ValueError("max timeout must be greater than 0")
        self._maxTimeoutSec = timeout


class OdriveAxis:

    _AXIS_STATUS_TOPIC  = "/{name}/controller_status"
    _AXIS_CONTROL_TOPIC = "/{name}/control_message"
    _AXIS_STATE_SRV     = "/{name}/request_axis_state"

    _STATE_CLIENT_MAX_TIMEOUT_SEC = 3.0
    
    _name: str #name of the axis
    _node: rclpy.node.Node # ROS node to use for publishers and subscribers
    _range: typing.Optional[AxisRange] #optional position range restriction (only enforced in position control mode)
    _conversionFactor: float #factor to convert motor rotations to mechanism rotations for each axis
    _status: TemporalValue[ControllerStatus] #current status of the axis controller
    _controlPublisher: rclpy.publisher.Publisher #publisher for sending control messages to the axis
    _stateClient: rclpy.client.Client #service client for setting the axis state
    _stateFuture: typing.Optional[rclpy.client.Future] #future for tracking the status of the most recent state change request
    _pos_offset:float
    _enforceCalibration: bool
    _isCalibrated: bool
    _inversion: int
    _currSetpoint: float


    def __init__(
            self,
            name: str,
            node: rclpy.node.Node,
            conversionFactor: float,
            range: typing.Optional[AxisRange],
            inverted: bool = False,
            enforceCalibration: bool = False
    ):
        self._name = name
        self._node = node
        self._conversionFactor = conversionFactor
        self._range = range
        self._status = TemporalValue[ControllerStatus](maxTimeoutSec=self._STATE_CLIENT_MAX_TIMEOUT_SEC)
        self._stateFuture = None
        self._pos_offset = 0
        self._enforceCalibration = enforceCalibration
        self._isCalibrated = False
        self._inversion = -1 if inverted else 1
        self._currSetpoint = math.nan

        self._node.create_subscription(
            ControllerStatus,
            self._AXIS_STATUS_TOPIC.format(name=name),
            self._status.update,
            10
        )

        self._controlPublisher = self._node.create_publisher(
            ControlMessage,
            self._AXIS_CONTROL_TOPIC.format(name=name),
            10
        )

        self._stateClient = self._node.create_client(
            odrive_can.srv.AxisState,
            self._AXIS_STATE_SRV.format(name=name)
        )

    def _set_state(self, state: AxisState):
        rq = odrive_can.srv.AxisState.Request()
        rq.axis_requested_state = state
        return self._stateClient.call_async(rq)
    

    def enable_axis(self) -> bool:
        if (self._stateFuture is not None) and (not self._stateFuture.done()):
            self._node.get_logger().warn(f"State change already in progress for axis {self._name}, cannot enable")
            return False
        self._stateFuture = self._set_state(AxisState.CLOSED_LOOP_CONTROL)
        return True


    def disable_axis(self) -> bool:
        if (self._stateFuture is not None) and (not self._stateFuture.done()):
            self._node.get_logger().warn(f"State change already in progress for axis {self._name}, cannot disable")
            return False
        self._stateFuture = self._set_state(AxisState.IDLE)
        self._currSetpoint = math.nan
        return True


    def get_velocity_deg_sec(self) -> typing.Optional[float]:
        if self._status.get_value() is None or self._status.max_timeout_exceeded():
            self._node.get_logger().warn(f"Axis {self._name} status is stale or not yet received, cannot get velocity")
            return None
        return self._inversion * (self._status.get_value().vel_estimate * self._conversionFactor * 360 / 60) # type: ignore


    def get_position_deg(self) -> typing.Optional[float]:
        if self._status.get_value() is None or self._status.max_timeout_exceeded():
            self._node.get_logger().warn(f"Axis {self._name} status is stale or not yet received, cannot get position")
            return None
        return self._inversion * ((self._status.get_value().pos_estimate * self._conversionFactor * 360) + self._pos_offset) # type: ignore
    

    def is_changing_state(self) -> bool:
        if self._stateFuture != None:
            return not self._stateFuture.done()
        
        return False


    def is_enabled(self) -> typing.Optional[bool]:
        if self._status.get_value() is None or self._status.max_timeout_exceeded() or self.is_changing_state():
            return False
        else:
            return self._status.get_value().axis_state == AxisState.CLOSED_LOOP_CONTROL

    #please set the axis to idle and reenable before calling if in velocity control mode. TODO: check for this condition
    def set_position(self, pos: float):
        if self._status.max_timeout_exceeded():
            self._node.get_logger().warn(f"Axis {self._name} controller status stale, position command ignored.")
            return
        
        if self._enforceCalibration and not self._isCalibrated:
            self._node.get_logger().warn(f"Axis {self._name} not calibrated, position command ignored.")
            return
        
        if self._range is not None:
            pos = self._range.clamp(pos)

        self._currSetpoint = pos

        msg = ControlMessage()
        msg.control_mode = ControlMode.POSITION_CONTROL
        msg.input_mode = InputMode.POS_FILTER
        msg.input_pos = ((self._inversion * pos - self._pos_offset) / self._conversionFactor / 360) #convert from mechanism degrees to motor revolutions
        self._controlPublisher.publish(msg)


    #please set the axis to idle and reenable before calling if in position control mode. TODO: check for this condition
    def set_velocity(self, vel: float):
        if self._status.max_timeout_exceeded():
            self._node.get_logger().warn(f"Axis {self._name} controller status stale, velocity command ignored.")
            return

        self._currSetpoint = vel

        msg = ControlMessage()
        msg.control_mode = ControlMode.VELOCITY_CONTROL
        msg.input_mode = InputMode.PASSTHROUGH
        msg.input_vel = self._inversion * ((vel / self._conversionFactor / 360) * 60) #convert from mechanism deg/s to motor revolutions/min
        self._controlPublisher.publish(msg)


    def reset_pos(self, new_pos_deg: float) -> bool:
        if self._status.get_value() is None or self._status.max_timeout_exceeded():
            return False
        
        curr_pos_deg = (self._status.get_value().pos_estimate * self._conversionFactor * 360)
        
        self._pos_offset = self._inversion * new_pos_deg - curr_pos_deg
        self._isCalibrated = True

        self._node.get_logger().info(f"Current position is: {curr_pos_deg}; New position offset is: {self._pos_offset}")
        return True


    def get_range_upper(self) -> float:
        if self._range != None:
            return self._range.get_max()
        else:
            return math.inf


    def get_range_lower(self) -> float:
        if self._range != None:
            return self._range.get_min()
        else:
            return -math.inf
        
    def is_calibrated(self) -> bool:
        return self._isCalibrated
    
    def calibration_enforced(self) -> bool:
        return self._enforceCalibration
    
    def get_setpoint(self) -> float:
        return float(self._currSetpoint)


# --------- HELPER FUNCTIONS ----------
def none_to_float_zero(input):
    if input is None:
        return 0.0
    else:
        return input   



# ---------- CONSTANTS ------------

#sensor topic names
TOWER_IMU_TOPIC     = "/gs_tower_imu/data"
TOWER_HEADING_TOPIC = "/gs_tower_imu/heading"
TOWER_GPS_TOPIC     = "/gs_tower_gps/fix"

ROVER_IMU_TOPIC     = "/imu/data"
ROVER_HEADING_TOPIC = "/imu/heading"
ROVER_GPS_TOPIC     = "/gps/fix"

#axis names
ELEV_AXIS_NAME = "elev_axis"
PAN_AXIS_NAME  = "pan_axis"

#topic names
MANUAL_CONTROL_TOPIC = "/gs_tower_control/manual_control_input"
CONTROL_STATUS_TOPIC = "/gs_tower_control/status"

CONTROL_SERVICE = "/gs_tower_control/service"


#factor to convert motor rotations to mechanism rotations for each axis
PAN_CONVERSION_FACTOR  = 1.0/56.63
ELEV_CONVERSION_FACTOR = 1.0/383.6
#PAN_CONVERSION_FACTOR = 1.0/100
#ELEV_CONVERSION_FACTOR = 1.0/100

#allowed angle ranges for each axis (deg)
#assumed that limits are a hard stop
PAN_ANGLE_RANGE  = AxisRange(-145, 145)
ELEV_ANGLE_RANGE = AxisRange(-26,  25)

#timeouts
ROVER_GPS_ALLOWED_TIMEOUT = 3.5
ROVER_IMU_ALLOWED_TIMEOUT = 3.0
TOWER_GPS_ALLOWED_TIMEOUT = 15.0
TOWER_IMU_ALLOWED_TIMEOUT = 15.0

TOWER_HEADING_MECHANICAL_OFFSET = 180

#physical dimensions
ROVER_HEIGHT_METERS = 3
TOWER_HEIGHT_METERS = 3

#axis velocity tolerances
ELEV_AXIS_VELOCITY_TOLERANCE_DEG_SEC = 1.0
PAN_AXIS_VELOCITY_TOLERANCE_DEG_SEC  = 1.0

#axis position tolerances
ELEV_AXIS_POSITION_TOLERANCE_DEG = 1.0
PAN_AXIS_POSITION_TOLERANCE_DEG  = 1.0

#homing motor spin up time
HOMING_START_TIME_SEC = 3
ELEV_HOMING_VELOCITY = 2.5 #deg/s
ELEV_HOMING_VELOCITY_THRESHOLD = 0.9 #homing is done when velocity is below THRESHOLD * VEL
ELEV_HOMING_STOP_POS = 27.5
PAN_HOMING_STOP_POS = 180.0
PAN_HOMING_VELOCITY = 1 #deg/s
PAN_HOMING_VELOCITY_THRESHOLD = 0.8

#control freq Hz
CONTROL_FREQ = 10.0

class AntennaControlMode(Enum):
    DISABLED = 0
    MANUAL_CONTROL = 1
    AUTOMATIC_CONTROL = 2
    HOMING = 3

class StatusFlags(Enum):
    NO_COMM_ELEV_AXIS      = 0b0000_0000_0000_0001
    NO_COMM_PAN_AXIS       = 0b0000_0000_0000_0010
    TIMEOUT_ROVER_GPS      = 0b0000_0000_0000_0100
    TIMEOUT_ROVER_IMU      = 0b0000_0000_0000_1000
    TIMEOUT_TOWER_GPS      = 0b0000_0000_0001_0000
    TIMEOUT_TOWER_IMU      = 0b0000_0000_0010_0000
    AXIS_UNCALIBRATED_PAN  = 0b0000_0000_0100_0000
    AXIS_UNCALIBRATED_ELEV = 0b0000_0000_1000_0000

class AntennaTowerControlNode(rclpy.node.Node):
        

    controlMode: AntennaControlMode
    is_calibrated: bool

    manualControlSubscriber: rclpy.subscription.Subscription
    statusPublisher: rclpy.publisher.Publisher

    rover_gps: TemporalValue[NavSatFix]
    tower_gps: TemporalValue[NavSatFix]

    rover_imu: TemporalValue[Imu]
    tower_imu: TemporalValue[Imu]

    rover_heading_corrected: TemporalValue[Float32]
    tower_heading_corrected: TemporalValue[Float32]

    manualControlInput: typing.Optional[AntennaControlManualInput]

    elev_axis: OdriveAxis
    pan_axis: OdriveAxis

    homing_timer: TemporalValue[bool]
    home_pan: bool
    home_elev: bool
    
    class HomingStep(Enum):
        ELEV_STATE_CHANGE = 0
        ELEV_AXIS_START = 1
        ELEV_AXIS_RUN = 2
        PAN_STATE_CHANGE = 3
        PAN_AXIS_START = 4
        PAN_AXIS_RUN = 5
        HOMING_CLEANUP = 6

    homingStep: HomingStep


    def positions_updated(self) -> bool:
        result = True

        if self.rover_gps.max_timeout_exceeded() or self.rover_gps.get_value() == None:
            result = False
            self.get_logger().warn("Unsafe for automatic control: rover gps timeout exceeded")
        if self.tower_gps.max_timeout_exceeded() or self.tower_gps.get_value() == None: 
            result = False
            self.get_logger().warn("Unsafe for automatic control: tower gps timeout exceeded")
        if self.rover_imu.max_timeout_exceeded() or self.rover_imu.get_value() == None:
            result = False
            self.get_logger().warn("Unsafe for automatic control: rover imu timeout exceeded")
        if self.tower_imu.max_timeout_exceeded() or self.tower_imu.get_value() == None:
            result = False
            self.get_logger().warn("Unsafe for automatic control: tower imu timeout exceeded")
        
        return result


    def control_service_callback(self, request: AntennaControlService.Request, response: AntennaControlService.Response):
        
        def str_to_ints(input: str):
            result = list[int]()
            for c in input:
                result.append(ord(c))

        response = AntennaControlService.Response()

        if (request.mode == AntennaControlMode.DISABLED.value):
            result =  self.elev_axis.disable_axis()
            result &= self.pan_axis.disable_axis()
            #if (not result):
            #    response.success = False
            #    return response
            response.success = True
            self.controlMode = AntennaControlMode.DISABLED

        elif (request.mode == AntennaControlMode.MANUAL_CONTROL.value):
            result =  self.elev_axis.enable_axis()
            result &= self.pan_axis.enable_axis()
            #if (not result):
            #    response.success = False
            #    return response
            response.success = True
            self.controlMode = AntennaControlMode.MANUAL_CONTROL

        elif (request.mode == AntennaControlMode.AUTOMATIC_CONTROL.value):

            if not self.positions_updated():
                response.success = False
                #response.msg = str_to_ints("Unsafe conditions for automatic control")
                return response

            result =  self.elev_axis.enable_axis()
            result &= self.pan_axis.enable_axis()
            #if (not result):
            #    response.success = False
            #    return response
            self.controlMode = AntennaControlMode.AUTOMATIC_CONTROL
            response.success = True
            #response.msg = str_to_ints("Automatic control not implemented")
            return response

        elif (request.mode == AntennaControlMode.HOMING.value):
            self.home_elev = False
            self.home_pan = False

            if self.elev_axis.get_position_deg() is not None:
                self.home_elev = True
                self.elev_axis.disable_axis()
            if self.pan_axis.get_position_deg() is not None:
                self.home_pan = True
                self.pan_axis.disable_axis()

            if self.home_pan and not self.home_elev:
                self.homingStep = self.HomingStep.PAN_STATE_CHANGE
            else:
                self.homingStep = self.HomingStep.ELEV_STATE_CHANGE

            self.controlMode = AntennaControlMode.HOMING
            response.success = True

        else:
            response.success = False
            #response.msg = str_to_ints("Invalid control mode")

        return response


    def rover_gps_callback(self, fix: NavSatFix):
        self.rover_gps.update(fix)


    def tower_gps_callback(self, fix: NavSatFix):
        self.tower_gps.update(fix)


    def publish_control_status(self):

        msg = AntennaControlStatus()

        msg.operating_mode = self.controlMode.value

        msg.errors = 0

        currentPanPos  = self.pan_axis.get_position_deg()
        currentPanVel  = self.pan_axis.get_velocity_deg_sec()
        currentElevPos = self.elev_axis.get_position_deg()
        currentElevVel = self.elev_axis.get_velocity_deg_sec()

        msg.current_elevation_deg = none_to_float_zero(currentElevPos)
        msg.current_pan_deg = none_to_float_zero(currentPanPos)
        msg.current_elevation_deg_sec = none_to_float_zero(currentElevVel)
        msg.current_pan_deg_sec = none_to_float_zero(currentPanVel)

        msg.current_pan_setpoint = self.pan_axis.get_setpoint()
        msg.current_elevation_setpoint = self.elev_axis.get_setpoint()

        # set status flags
        if currentPanPos is None or currentPanVel is None:
            msg.errors |= StatusFlags.NO_COMM_PAN_AXIS.value

        if currentElevPos is None or currentElevVel is None:
            msg.errors |= StatusFlags.NO_COMM_ELEV_AXIS.value

        if self.rover_gps.max_timeout_exceeded():
            msg.errors |= StatusFlags.TIMEOUT_ROVER_GPS.value

        if self.rover_imu.max_timeout_exceeded():
            msg.errors |= StatusFlags.TIMEOUT_ROVER_IMU.value

        if self.tower_gps.max_timeout_exceeded():
            msg.errors |= StatusFlags.TIMEOUT_TOWER_GPS.value

        if self.tower_imu.max_timeout_exceeded():
            msg.errors |= StatusFlags.TIMEOUT_TOWER_IMU.value

        if not self.pan_axis.is_calibrated():
            msg.errors |= StatusFlags.AXIS_UNCALIBRATED_PAN.value

        if not self.elev_axis.is_calibrated():
            msg.errors |= StatusFlags.AXIS_UNCALIBRATED_ELEV.value

        #TODO add checking for setpoint status

        self.statusPublisher.publish(msg)


    def execute_automatic_control(self):

        def quat_to_LatLong(quat):
            rot = Rotation.from_quat(quat)
            euler = rot.as_euler("zyx", True)
            return LatLong(1, euler[1], euler[0])
        
        def normalize_angle(angle: float):
            while angle < -180:
                angle += 360
            while angle > 180:
                angle -= 360
            return angle
            
        if not self.positions_updated():
            self.elev_axis.disable_axis()
            self.pan_axis.disable_axis()
            self.controlMode = AntennaControlMode.DISABLED
            return
            
        try:
            rawLoc = self.rover_gps.get_value()
            roverLoc = LatLong(
                EARTH_RADIUS_M + rawLoc.altitude,
                rawLoc.latitude,
                rawLoc.longitude
            )

            #rawLoc = self.rover_gps.get_value()
            #roverBaseLoc = LatLong(
            #    EARTH_RADIUS_M + rawLoc.altitude,
            #    rawLoc.latitude,
            #    rawLoc.longitude
            #)
            #roverAntennaOffset = quat_to_LatLong(
            #        [
            #            self.rover_imu.get_value().orientation.x,
            #            self.rover_imu.get_value().orientation.y,
            #            self.rover_imu.get_value().orientation.z,
            #            self.rover_imu.get_value().orientation.w
            #        ]
            #    ) * ROVER_HEIGHT_METERS
            #self.get_logger().info(f"Rover Ant. Offset {roverAntennaOffset}")
            #roverLoc = roverBaseLoc + roverAntennaOffset


            rawLoc = self.tower_gps.get_value()
            towerLoc = LatLong(
                EARTH_RADIUS_M + rawLoc.altitude,
                rawLoc.latitude,
                rawLoc.longitude
            )

            #towerBaseLoc = LatLong(
            #    EARTH_RADIUS_M + rawLoc.altitude,
            #    rawLoc.latitude,
            #    rawLoc.longitude
            #)
            #towerAntennaOffset = quat_to_LatLong(
            #    [
            #        self.tower_imu.get_value().orientation.x,
            #        self.tower_imu.get_value().orientation.y,
            #        self.tower_imu.get_value().orientation.z,
            #        self.tower_imu.get_value().orientation.w
            #    ]
            #) * TOWER_HEIGHT_METERS
            #self.get_logger().info(f"Tower Ant. Offset {towerAntennaOffset}")
            #towerLoc = towerBaseLoc + towerAntennaOffset

            self.get_logger().info(f"Computed new angles: \n Pan: {getPanAngleDegrees(towerLoc, roverLoc)} \n Tilt: {getElevationAngleDegrees(towerLoc, roverLoc)}")

            self.pan_axis.set_position(normalize_angle(getPanAngleDegrees(towerLoc, roverLoc) - (self.tower_heading.get_value().data + TOWER_HEADING_MECHANICAL_OFFSET)))
            self.elev_axis.set_position(getElevationAngleDegrees(towerLoc, roverLoc))
        except Exception as e:
            self.get_logger().warn(f"Failed to compute new angles due to exception: {e}")


    def manual_control_input_callback(self, data: AntennaControlManualInput):
        self.get_logger().info(f"Recieved manual control input: p: {data.pan_deg}; e: {data.elevation_deg}")
        self.manualControlInput = data


    def execute_manual_control(self):
        if self.manualControlInput is not None:
            self.elev_axis.set_position(self.manualControlInput.elevation_deg)
            self.pan_axis.set_position(self.manualControlInput.pan_deg)


    def execute_homing(self):
        #self.pan_axis.reset_pos(0) #TODO Make this code into a separate reset mode to allow zeroing the antenna without homing
        #self.elev_axis.reset_pos(0)
        #if self.manualControlInput is not None:
        #    self.manualControlInput.elevation_deg = 0.0
        #    self.manualControlInput.pan_deg = 0.0
        #self.get_logger().info("Positions reset")
        #self.controlMode = AntennaControlMode.DISABLED

        match self.homingStep:
            case self.HomingStep.ELEV_STATE_CHANGE:
                if self.elev_axis.is_changing_state():
                    return
                elif not self.elev_axis.is_enabled():
                    self.elev_axis.enable_axis()
                else:
                    self.get_logger().info("Starting elevation axis homing sequence")
                    self.homingStep = self.HomingStep.ELEV_AXIS_START
                    self.homing_timer.update(False)
            case self.HomingStep.ELEV_AXIS_START:
                if self.homing_timer.get_value() == False:
                    self.homing_timer.update(True)
                    self.elev_axis.set_velocity(ELEV_HOMING_VELOCITY)
                elif self.homing_timer.max_timeout_exceeded():
                    self.homingStep = self.HomingStep.ELEV_AXIS_RUN
            case self.HomingStep.ELEV_AXIS_RUN:
                vel = self.elev_axis.get_velocity_deg_sec()
                if vel is None:
                    self.elev_axis.disable_axis()
                    self.homingStep = self.HomingStep.PAN_STATE_CHANGE if self.home_pan else self.HomingStep.HOMING_CLEANUP
                    self.get_logger().error("Communication lost with elevation axis, homing of axis aborted.")
                elif abs(vel) < abs(ELEV_HOMING_VELOCITY * ELEV_HOMING_VELOCITY_THRESHOLD):
                    self.elev_axis.disable_axis()
                    self.elev_axis.reset_pos(ELEV_HOMING_STOP_POS)
                    self.get_logger().info("Homing of elevation axis completed successfully")
                    self.homingStep = self.HomingStep.PAN_STATE_CHANGE if self.home_pan else self.HomingStep.HOMING_CLEANUP

            case self.HomingStep.PAN_STATE_CHANGE:
                if self.pan_axis.is_changing_state():
                    return
                elif not self.pan_axis.is_enabled():
                    self.pan_axis.enable_axis()
                else:
                    self.get_logger().info("Starting pan axis homing sequence")
                    self.homingStep = self.HomingStep.PAN_AXIS_START
                    self.homing_timer.update(False)
            case self.HomingStep.PAN_AXIS_START:
                if self.homing_timer.get_value() == False:
                    self.homing_timer.update(True)
                    self.pan_axis.set_velocity(PAN_HOMING_VELOCITY)
                elif self.homing_timer.max_timeout_exceeded():
                    self.homingStep = self.HomingStep.PAN_AXIS_RUN
            case self.HomingStep.PAN_AXIS_RUN:
                vel = self.pan_axis.get_velocity_deg_sec()
                if vel is None:
                    self.pan_axis.disable_axis()
                    self.homingStep = self.HomingStep.HOMING_CLEANUP
                    self.get_logger().error("Communication lost with pan axis, homing of axis aborted.")
                elif abs(vel) < abs(PAN_HOMING_VELOCITY * PAN_HOMING_VELOCITY_THRESHOLD):
                    self.pan_axis.disable_axis()
                    self.pan_axis.reset_pos(PAN_HOMING_STOP_POS)
                    self.get_logger().info("Homing of pan axis completed successfully")
                    self.homingStep = self.HomingStep.HOMING_CLEANUP
            
            case self.HomingStep.HOMING_CLEANUP:
                self.elev_axis.disable_axis()
                self.pan_axis.disable_axis()
                self.controlMode = AntennaControlMode.DISABLED
                pass
    

    def control_loop(self):
        #execute control depending on state
        if self.controlMode == AntennaControlMode.MANUAL_CONTROL:
            self.execute_manual_control()
        elif self.controlMode == AntennaControlMode.AUTOMATIC_CONTROL:
            self.execute_automatic_control()
        elif self.controlMode == AntennaControlMode.HOMING:
            self.execute_homing()
        
        self.publish_control_status()

    
    def __init__(self):
        super().__init__(node_name="gs_tower")

        self.rover_gps = TemporalValue[NavSatFix](maxTimeoutSec=ROVER_GPS_ALLOWED_TIMEOUT)
        self.tower_gps = TemporalValue[NavSatFix](maxTimeoutSec=TOWER_GPS_ALLOWED_TIMEOUT)

        self.rover_imu = TemporalValue[Imu](maxTimeoutSec=ROVER_IMU_ALLOWED_TIMEOUT)
        self.tower_imu = TemporalValue[Imu](maxTimeoutSec=TOWER_IMU_ALLOWED_TIMEOUT)

        self.rover_heading = TemporalValue[Float32](maxTimeoutSec=ROVER_IMU_ALLOWED_TIMEOUT)
        self.tower_heading = TemporalValue[Float32](maxTimeoutSec=TOWER_IMU_ALLOWED_TIMEOUT)

        self.homing_timer = TemporalValue[bool](False, maxTimeoutSec=HOMING_START_TIME_SEC)

        self.manualControlInput = None


        #rover and tower gps subscriptions
        self.create_subscription(
            NavSatFix,
            ROVER_GPS_TOPIC,
            self.rover_gps_callback,
            10
        )

        self.create_subscription(
            NavSatFix,
            TOWER_GPS_TOPIC,
            self.tower_gps_callback,
            10
        )

        #imu and heading subscribers
        self.create_subscription(
            Imu,
            TOWER_IMU_TOPIC,
            self.tower_imu.update,
            10
        )

        self.create_subscription(
            Imu,
            ROVER_IMU_TOPIC,
            self.rover_imu.update,
            10
        )

        self.create_subscription(
            Float32,
            TOWER_HEADING_TOPIC,
            self.tower_heading.update,
            10
        )

        self.create_subscription(
            Float32,
            ROVER_HEADING_TOPIC,
            self.rover_heading.update,
            10
        )

        self.manualControlSubscriber = self.create_subscription(
            AntennaControlManualInput,
            MANUAL_CONTROL_TOPIC,
            self.manual_control_input_callback,
            10
        )

        #mode service handler
        self.create_service(
            AntennaControlService,
            CONTROL_SERVICE,
            self.control_service_callback
        )

        #main loop timer
        self.create_timer(
            1 / CONTROL_FREQ,
            self.control_loop
        )

        #status publisher
        self.statusPublisher = self.create_publisher(
            AntennaControlStatus,
            CONTROL_STATUS_TOPIC,
            10
        )

        #define variables
        self.elev_axis = OdriveAxis(
            ELEV_AXIS_NAME,
            self,
            ELEV_CONVERSION_FACTOR,
            ELEV_ANGLE_RANGE
        )

        self.pan_axis = OdriveAxis(
            PAN_AXIS_NAME,
            self,
            PAN_CONVERSION_FACTOR,
            PAN_ANGLE_RANGE,
            inverted=True
        )

        #initialize axes to idle mode and control to disabled
        self.elev_axis.disable_axis()
        self.pan_axis.disable_axis()
        self.controlMode = AntennaControlMode.DISABLED
        self.is_calibrated = False
        self.elev_calibrated = False
        self.pan_calibrated = False


def main(args=None):

    try: 
        rclpy.init(args=args)
        
        rclpy.spin(
            AntennaTowerControlNode(),
        )

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        rclpy.shutdown()
        pass
    


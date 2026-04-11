from copy import deepcopy
from telnetlib import EL
from tkinter import DISABLED
from tkinter.tix import Control

import math
import time
import typing
from enum import Enum
from typing import Callable, Sequence, TypeVar, Generic
from odrive.enums import ControlMode, InputMode
from odrive.enums import AxisState
from gs_tower_control.coordinate_math import *
from gs_tower_interfaces.msg import AntennaControlStatus, AntennaControlManualInput
from gs_tower_interfaces.srv import AntennaControlService
from odrive_can.msg import ODriveStatus, ControlMessage, ControllerStatus
import odrive_can.srv
import rclpy.client
import rclpy.executors
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
    _conversionFactor: float #factor to convert from motor revolutions to degrees for this axis
    _status: TemporalValue[ControllerStatus] #current status of the axis controller
    _controlPublisher: rclpy.publisher.Publisher #publisher for sending control messages to the axis
    _stateClient: rclpy.client.Client #service client for setting the axis state
    _stateFuture: typing.Optional[rclpy.client.Future] #future for tracking the status of the most recent state change request

    def __init__(
            self,
            name: str,
            node: rclpy.node.Node,
            conversionFactor: float,
            range: typing.Optional[AxisRange]
    ):
        self._name = name
        self._node = node
        self._conversionFactor = conversionFactor
        self._range = range
        self._status = TemporalValue[ControllerStatus](maxTimeoutSec=self._STATE_CLIENT_MAX_TIMEOUT_SEC)
        self._stateFuture = None

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
    

    def enable_axis(self):
        if (self._stateFuture is not None) and (not self._stateFuture.done()):
            self._node.get_logger().warn(f"State change already in progress for axis {self._name}, cannot enable")
            return
        self._stateFuture = self._set_state(AxisState.CLOSED_LOOP_CONTROL)


    def disable_axis(self):
        if (self._stateFuture is not None) and (not self._stateFuture.done()):
            self._node.get_logger().warn(f"State change already in progress for axis {self._name}, cannot disable")
            return
        self._stateFuture = self._set_state(AxisState.IDLE)


    def get_velocity_deg_sec(self) -> typing.Optional[float]:
        if self._status.get_value() is None or self._status.max_timeout_exceeded():
            self._node.get_logger().warn(f"Axis {self._name} status is stale or not yet received, cannot get velocity")
            return None
        return self._status.get_value().vel_estimate / self._conversionFactor / 60 # type: ignore


    def get_position_deg(self) -> typing.Optional[float]:
        if self._status.get_value() is None or self._status.max_timeout_exceeded():
            self._node.get_logger().warn(f"Axis {self._name} status is stale or not yet received, cannot get position")
            return None
        return self._status.get_value().pos_estimate / self._conversionFactor # type: ignore


    #please set the axis to idle and reenable before calling if in vecocity control mode. TODO: check for this condition
    def set_position(self, pos: float):
        if self._range is not None:
            pos = self._range.clamp(pos)

        msg = ControlMessage()
        msg.control_mode = ControlMode.POSITION_CONTROL
        msg.input_mode = InputMode.PASSTHROUGH
        msg.input_pos = pos * self._conversionFactor #convert from revolutions to degrees
        self._controlPublisher.publish(msg)


    #please set the axis to idle and reenable before calling if in position control mode. TODO: check for this condition
    def set_velocity(self, vel: float):
        msg = ControlMessage()
        msg.control_mode = ControlMode.VELOCITY_CONTROL
        msg.input_mode = InputMode.PASSTHROUGH
        msg.input_vel = vel / self._conversionFactor / 60 #convert from deg/s to motor revolutions/min
        self._controlPublisher.publish(msg)

    



# ---------- CONSTANTS ------------

#sensor topic names
TOWER_IMU_TOPIC     = "gs_tower_imu/data"
TOWER_HEADING_TOPIC = "gs_tower_imu/heading"
TOWER_GPS_TOPIC     = "gs_tower_gps/fix"

ROVER_IMU_TOPIC     = "imu/data"
ROVER_HEADING_TOPIC = "imu/heading"
ROVER_GPS_TOPIC     = "gps/fix"

#axis names
ELEV_AXIS_NAME = "elev_axis"
PAN_AXIS_NAME  = "pan_axis"

#topic names
MANUAL_CONTROL_TOPIC = "/gs_tower_control/manual_control_input"
CONTROL_STATUS_TOPIC = "/gs_tower_control/status"


#factor to convert motor rotations to degrees for each axis
PAN_CONVERSION_FACTOR  = 1.0
ELEV_CONVERSION_FACTOR = 1.0

#allowed angle ranges for each axis (deg)
PAN_ANGLE_RANGE  = AxisRange(-170, 170)
ELEV_ANGLE_RANGE = AxisRange( -30,  30)

#timer intervals
STATUS_PUBLISHER_INTERVAL_SEC = 0.25


class AntennaTowerControlNode(rclpy.node.Node):

    class AntennaControlMode(Enum):
        DISABLED = 0
        MANUAL_CONTROL = 1
        AUTOMATIC_CONTROL = 2
        HOMING = 3

    controlMode: AntennaControlMode
    heading_offset: float
    is_calibrated: bool

    manualControlSubscriber: rclpy.subscription.Subscription
    statusPublisher: rclpy.publisher.Publisher

    rover_gps: TemporalValue[NavSatFix]
    tower_gps: TemporalValue[NavSatFix]

    rover_imu: TemporalValue[Imu]
    tower_imu: TemporalValue[Imu]

    rover_heading_corrected: TemporalValue[Float32]
    tower_heading_corrected: TemporalValue[Float32]

    elev_axis: OdriveAxis
    pan_axis: OdriveAxis

    def control_service_callback(self, request: AntennaControlService.Request, response: AntennaControlService.Response):
        
        def str_to_ints(input: str):
            result = list[int]()
            for c in input:
                result.append(ord(c))

        response = AntennaControlService.Response()

        if (request.mode == self.AntennaControlMode.DISABLED.value):
            self.controlMode = self.AntennaControlMode.DISABLED
            self.elev_axis.disable_axis()
            self.pan_axis.disable_axis()
            response.success = True

        elif (request.mode == self.AntennaControlMode.MANUAL_CONTROL.value):
            self.controlMode = self.AntennaControlMode.MANUAL_CONTROL
            self.elev_axis.enable_axis()
            self.pan_axis.enable_axis()
            response.success = True
            #TODO: check for homing

        elif (request.mode == self.AntennaControlMode.AUTOMATIC_CONTROL.value):
            response.success = False
            #response.msg = str_to_ints("Automatic control not implemented")

        elif (request.mode == self.AntennaControlMode.HOMING.value):
            pass

        else:
            response.success = False
            #response.msg = str_to_ints("Invalid control mode")

        return response


    def rover_gps_callback(self, fix: NavSatFix):
        self.rover_gps.update(fix)
        if (self.controlMode == self.AntennaControlMode.AUTOMATIC_CONTROL):
            self.execute_automatic_control()


    def control_status_publisher_callback(self):
        def none_to_float_zero(input):
            if input is None:
                return 0.0
            else:
                return input

        msg = AntennaControlStatus()

        msg.operating_mode = self.controlMode.value

        msg.current_elevation_deg = none_to_float_zero(self.elev_axis.get_position_deg())
        msg.current_pan_deg = none_to_float_zero(self.pan_axis.get_position_deg())
        msg.current_elevation_deg_sec = none_to_float_zero(self.pan_axis.get_velocity_deg_sec())
        msg.current_pan_deg_sec = none_to_float_zero(self.pan_axis.get_velocity_deg_sec())
        #TODO add checking for setpoint status

        self.statusPublisher.publish(msg)


    def execute_automatic_control(self):
        pass


    def execute_manual_control(self, data: AntennaControlManualInput):
        if self.controlMode == self.AntennaControlMode.MANUAL_CONTROL:
            #TODO: ensure axis controller is sucessfully enabled before trying to set position
            self.elev_axis.set_position(data.elevation_deg)
            self.pan_axis.set_position(data.pan_deg)


    def execute_homing(self):
        pass


    def start_homing(self):
        pass


    def apply_heading_correction(self, hdg: Float32) -> Float32:
        result = Float32()
        result.data = hdg.data + self.heading_offset
        return result

    def __init__(self):
        super().__init__(node_name="gs_tower")

        self.rover_gps = TemporalValue[NavSatFix]()
        self.tower_gps = TemporalValue[NavSatFix]()

        self.rover_imu = TemporalValue[Imu]()
        self.tower_imu = TemporalValue[Imu]()

        self.rover_heading_corrected = TemporalValue[Float32]()
        self.tower_heading_corrected = TemporalValue[Float32]()

        self.elev_axis_status = TemporalValue[ControllerStatus]()
        self.pan_axis_status  = TemporalValue[ControllerStatus]()

        self.heading_offset = getMagneticNorthOffsetDegrees()


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
            self.tower_gps.update,
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
            lambda h: self.tower_heading_corrected.update(self.apply_heading_correction(h)),
            10
        )

        self.create_subscription(
            Float32,
            ROVER_HEADING_TOPIC,
            lambda h: self.rover_heading_corrected.update(self.apply_heading_correction(h)),
            10
        )

        self.manualControlSubscriber = self.create_subscription(
            AntennaControlManualInput,
            MANUAL_CONTROL_TOPIC,
            self.execute_manual_control,
            10
        )

        #mode service handler
        self.create_service(
            AntennaControlService,
            "gs_antenna_control",
            self.control_service_callback
        )

        #status publishing timer
        self.create_timer(
            STATUS_PUBLISHER_INTERVAL_SEC,
            self.control_status_publisher_callback
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
            PAN_ANGLE_RANGE
        )

        #initialize axes to idle mode and control to disabled
        self.elev_axis.disable_axis()
        self.pan_axis.disable_axis()
        self.controlMode = self.AntennaControlMode.DISABLED
        self.is_calibrated = False

def main(args=None):

    try: 
        rclpy.init(args=args)
        
        rclpy.spin(
            AntennaTowerControlNode(),
        )

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        rclpy.shutdown()
        pass
    


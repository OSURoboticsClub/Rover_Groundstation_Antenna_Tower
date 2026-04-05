from tkinter import DISABLED
from tkinter.tix import Control

import math
import time
import typing
from enum import Enum
from typing import Callable, TypeVar, Generic
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
    



# ---------- CONSTANTS ------------

#sensor topic names
TOWER_IMU_TOPIC     = "gs_tower_imu/data"
TOWER_HEADING_TOPIC = "gs_tower_imu/heading"
TOWER_GPS_TOPIC     = "gs_tower_gps/fix"

ROVER_IMU_TOPIC     = "imu/data"
ROVER_HEADING_TOPIC = "imu/heading"
ROVER_GPS_TOPIC     = "gps/fix"

ELEV_AXIS_STATUS_TOPIC  = "/elev_axis/controller_status"
ELEV_AXIS_CONTROL_TOPIC = "/elev_axis/control_message"
ELEV_AXIS_STATE_SRV     = "/elev_axis/request_axis_state"

PAN_AXIS_STATUS_TOPIC   = "/pan_axis/controller_status"
PAN_AXIS_CONTROL_TOPIC  = "/pan_axis/control_message"
PAN_AXIS_STATE_SRV      = "/pan_axis/request_axis_state"

#topic names
MANUAL_CONTROL_TOPIC = "/gs_tower_control/manual_control_input"


#factor to convert motor rotations to degrees for each axis
PAN_CONVERSION_FACTOR  = 1.0
ELEV_CONVERSION_FACTOR = 1.0

#allowed angle ranges for each axis (deg)
PAN_ANGLE_RANGE  = AxisRange(-170, 170)
ELEV_ANGLE_RANGE = AxisRange( -30,  30)


class AntennaTowerControlNode(rclpy.node.Node):

    class AntennaControlMode(Enum):
        DISABLED = 0,
        MANUAL_CONTROL = 1,
        AUTOMATIC_CONTROL = 2,
        HOMING = 3

    controlMode: AntennaControlMode
    heading_offset: float
    is_calibrated: bool

    elevAxisControlPublisher: rclpy.publisher.Publisher
    panAxisControlPublisher:  rclpy.publisher.Publisher
    elevAxisStateClient: rclpy.client.Client
    panAxisStateClient:  rclpy.client.Client
    manualControlSubscriber: rclpy.subscription.Subscription

    rover_gps: TemporalValue[NavSatFix]
    tower_gps: TemporalValue[NavSatFix]

    rover_imu: TemporalValue[Imu]
    tower_imu: TemporalValue[Imu]

    rover_heading_corrected: TemporalValue[Float32]
    tower_heading_corrected: TemporalValue[Float32]

    elev_axis_status: TemporalValue[ControllerStatus]
    pan_axis_status:  TemporalValue[ControllerStatus]

    def set_axis_states(self, state: AxisState):
        rq = odrive_can.srv.AxisState.Request()
        rq.axis_requested_state = state
        self.elevAxisStateClient.call(rq)
        self.panAxisStateClient.call(rq)

    def control_service_callback(self, request: AntennaControlService.Request, response: AntennaControlService.Response):
        response = AntennaControlService.Response()

        if (request.mode == self.AntennaControlMode.DISABLED):
            self.controlMode = self.AntennaControlMode.DISABLED

        elif (request.mode == self.AntennaControlMode.MANUAL_CONTROL):
            self.controlMode = self.AntennaControlMode.MANUAL_CONTROL
            self.set_axis_states(AxisState.CLOSED_LOOP_CONTROL)
            #TODO: check for homing

        elif (request.mode == self.AntennaControlMode.AUTOMATIC_CONTROL):
            response.success = False
            response.msg = "Automatic control not implemented"

        elif (request.mode == self.AntennaControlMode.HOMING):
            pass

        else:
            response.success = False
            response.msg = "Invalid control mode"


    def rover_gps_callback(self, fix: NavSatFix):
        self.rover_gps.update(fix)
        if (self.controlMode == self.AntennaControlMode.AUTOMATIC_CONTROL):
            self.execute_automatic_control()


    def execute_automatic_control(self):
        pass


    def execute_manual_control(self, data: AntennaControlManualInput):
        if self.controlMode == self.AntennaControlMode.MANUAL_CONTROL:
            elevInput = ControlMessage()
            elevInput.control_mode = ControlMode.POSITION_CONTROL
            elevInput.input_mode = InputMode.PASSTHROUGH
            #convert from deg to motor revolutions
            elevInput.input_pos = data.elevation_deg/360/ELEV_CONVERSION_FACTOR
            self.elevAxisControlPublisher.publish(elevInput)
            panInput = ControlMessage()
            panInput.control_mode = ControlMode.POSITION_CONTROL
            panInput.input_mode = InputMode.PASSTHROUGH
            #convert from deg to motor revolutions
            panInput.input_pos = data.pan_deg/360/PAN_CONVERSION_FACTOR
            self.panAxisControlPublisher.publish(panInput)


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

        #publishers for Odrive control messages
        self.elevAxisControlPublisher = self.create_publisher(
            ControlMessage,
            ELEV_AXIS_CONTROL_TOPIC,
            10
        )

        self.panAxisControlPublisher = self.create_publisher(
            ControlMessage,
            PAN_AXIS_CONTROL_TOPIC,
            10
        )

        #services clients for setting Odrive axis state
        self.elevAxisStateClient = self.create_client(
            odrive_can.srv.AxisState,
            ELEV_AXIS_STATE_SRV
        )

        self.panAxisStateClient = self.create_client(
            odrive_can.srv.AxisState,
            PAN_AXIS_STATE_SRV
        )

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

        #axis status subscribers
        self.create_subscription(
            ControllerStatus,
            ELEV_AXIS_STATUS_TOPIC,
            self.elev_axis_status.update,
            10
        )

        self.create_subscription(
            ControllerStatus,
            PAN_AXIS_STATUS_TOPIC,
            self.elev_axis_status.update,
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

        #define variables


        #initialize axes to idle mode and control to disabled
        self.set_axis_states(AxisState.IDLE)
        self.controlMode = self.AntennaControlMode.DISABLED
        self.is_calibrated = False

def main(args=None):

    try: 
        rclpy.init(args=args)
        
        rclpy.spin(
            AntennaTowerControlNode(),
            rclpy.executors.Executor()
        )

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        rclpy.shutdown()
        pass
    


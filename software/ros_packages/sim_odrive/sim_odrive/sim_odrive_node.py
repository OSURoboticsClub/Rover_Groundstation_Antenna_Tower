import rclpy
import rclpy.node
import rclpy.executors
import typing
import time
import math
from odrive_can.msg import ODriveStatus, ControlMessage, ControllerStatus
from odrive_can.srv import AxisState
from odrive.enums import ControlMode, InputMode
import odrive.enums

class SimOdriveNode(rclpy.node.Node):

    AXIS_STATUS_TOPIC  = "{NAME}/controller_status"
    AXIS_CONTROL_TOPIC = "{NAME}/control_message"
    AXIS_STATE_SRV     = "{NAME}/request_axis_state"
    UPDATE_FREQ_HZ     = 20

    def controller_status_callback(self, data: ControlMessage):
        self.controlMessage = data


    def axis_state_service(self, request: AxisState.Request, response: AxisState.Response):
        response = AxisState.Response()
        response.axis_state = request.axis_requested_state
        response.procedure_result = 1
        if request.axis_requested_state == odrive.enums.AxisState.CLOSED_LOOP_CONTROL.value or request.axis_requested_state == odrive.enums.AxisState.IDLE.value:
            self.axisState = request.axis_requested_state
            self.get_logger().info(f"Axis state set to {self.axisState}.")
            response.procedure_result = 0
        else:
            self.get_logger().warn(f"Axis state {request.axis_requested_state} unsupported.")
        
        return response


    def publish_controller_status(self):
        status = ControllerStatus()
        status.pos_estimate = float(self.motorPosition)
        status.vel_estimate = float(self.motorVelocity)
        status.axis_state   = self.axisState

        self.statusPublisher.publish(status)

    
    def run(self):
        now = time.time()
        deltaTime = now - self.updateTime
        self.updateTime = now
        if self.axisState == odrive.enums.AxisState.CLOSED_LOOP_CONTROL.value:
            if self.controlMessage is not None and self.controlMessage.control_mode == ControlMode.VELOCITY_CONTROL.value:
                self.motorVelocity = self.controlMessage.input_vel
            elif self.controlMessage is not None and self.controlMessage.control_mode == ControlMode.POSITION_CONTROL.value:
                if math.isclose(self.motorPosition, self.controlMessage.input_pos, abs_tol=0.05):
                    self.motorVelocity = 0
                elif self.motorPosition < self.controlMessage.input_pos:
                    self.motorVelocity = self.maxVelocity
                else:
                    self.motorVelocity = -self.maxVelocity
            self.motorPosition += (self.motorVelocity / 60) * deltaTime

        self.publish_controller_status()
            
    
    def __init__(self):

        super().__init__('sim_odrive_node')

        self.controlMessage: typing.Optional[ControlMessage] = None
        self.axisState = odrive.enums.AxisState.IDLE.value
        self.motorPosition = 0
        self.motorVelocity = 0
        self.updateTime = time.time()

        self.maxVelocityParam = self.declare_parameter("maxVelocity", 500.0)
        self.maxVelocity = self.maxVelocityParam.get_parameter_value().double_value
        self.get_logger().info(f"Maximum velocity set to {self.maxVelocity} rpm")

        self.statusPublisher = self.create_publisher(
            ControllerStatus,
            self.AXIS_STATUS_TOPIC.format(NAME=self.get_namespace()),
            10
        )

        self.controlSubscriber = self.create_subscription(
            ControlMessage,
            self.AXIS_CONTROL_TOPIC.format(NAME=self.get_namespace()),
            self.controller_status_callback,
            10
        )

        self.create_timer(
            1/self.UPDATE_FREQ_HZ,
            self.run
        )

        self.create_service(
            AxisState,
            self.AXIS_STATE_SRV.format(NAME=self.get_namespace()),
            self.axis_state_service
        )


def main(args=None):

    try: 
        rclpy.init(args=args)
        
        rclpy.spin(
            SimOdriveNode(),
        )

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        rclpy.shutdown()
        pass




if __name__ == "__main__":
    main()
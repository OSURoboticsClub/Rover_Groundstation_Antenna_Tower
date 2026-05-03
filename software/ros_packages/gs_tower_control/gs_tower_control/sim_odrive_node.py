import rclpy
import typing
from odrive_can.msg import ODriveStatus, ControlMessage, ControllerStatus
from odrive_can.srv import AxisState
from odrive.enums import ControlMode, InputMode
import odrive.enums

class SimOdriveNode(rclpy.Node):

    AXIS_STATUS_TOPIC  = "/{name}/controller_status"
    AXIS_CONTROL_TOPIC = "/{name}/control_message"
    AXIS_STATE_SRV     = "/{name}/request_axis_state"
    UPDATE_FREQ_HZ     = 20

    def controller_status_callback(self, data: ControlMessage):
        self.controlMessage = data

    def axis_state_service(self, request: AxisState.Request, response: AxisState.Response):
        response = AxisState.Response()
        response.axis_state = request.axis_requested_state
        if request.axis_requested_state == odrive.enums.AxisState.CLOSED_LOOP_CONTROL.value or request.axis_requested_state == odrive.enums.AxisState.IDLE.value:
            self.axisState = odrive.enums.AxisState.value
            self.get_logger().info(f"Axis state set to {self.axisState}.")
        else:
            self.get_logger().warn(f"Axis state {request.axis_requested_state} unsupported.")

    
    def run(self):
        if self.axisState == odrive.enums.AxisState.CLOSED_LOOP_CONTROL.value:
            if self.controlMode == ControlMode.VELOCITY_CONTROL:
                pass
            elif self.controlMode == ControlMode.POSITION_CONTROL:
                pass
            
    
    def __init__(self):

        self.controlMessage: typing.Optional[ControlMessage] = None
        self.controlMode = None
        self.axisState = odrive.enums.AxisState.IDLE.value

        NAME = self.declare_parameter("name", "sim_odrive")
        self.maxVelocity = self.declare_parameter("maxVelocity", 5)

        self.statusPublisher = self.create_publisher(
            ControllerStatus,
            self.AXIS_STATUS_TOPIC.format(NAME),
            10
        )

        self.controlSubscriber = self.create_subscription(
            ControlMessage,
            self.AXIS_CONTROL_TOPIC.format(NAME),
            self.controller_status_callback,
            10
        )


def main():
    pass




if __name__ == "__main__":
    main()
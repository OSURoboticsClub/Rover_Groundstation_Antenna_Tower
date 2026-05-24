import copy
import rclpy
import time
import rclpy.node
from gs_tower_interfaces.msg import AntennaControlStatus
from gs_tower_interfaces.msg import AntennaControlManualInput
from gs_tower_interfaces.srv import AntennaControlService
import gs_tower_control.control_script as control


class CommsUINode(rclpy.node.Node):

    STATUS_TIMEOUT = 5
    _statusTime = time.time()

    def get_status(self):
        if (time.time() - self._statusTime > self.STATUS_TIMEOUT):
            return None
        
        return copy.deepcopy(self._status)
    

    def status_callback(self, data: AntennaControlStatus):
        self._statusTime = time.time()
        self._status = data


    def publish_manual_control_input(self, panAngle: float, elevAngle: float):
        msg = AntennaControlManualInput()
        msg.elevation_deg = elevAngle
        msg.pan_deg = panAngle

        self._manualControlPublisher.publish(msg)

    
    def set_mode(self, mode: control.AntennaControlMode) -> bool:
        request = AntennaControlService.Request()
        request.mode = mode.value
        result = self._antennaControlClient.call_async(request)
        rclpy.spin_until_future_complete(self, result)

        return result.result().success
    

    def __init__(self):

        self._status = None

        super().__init__("comms_ui_node")

        self._manualControlPublisher = self.create_publisher (
            AntennaControlManualInput,
            "/gs_tower_control/manual_control_input",
            10
        )

        self._statusSubscriber = self.create_subscription (
            AntennaControlStatus,
            "/gs_tower_control/status",
            self.status_callback,
            10
        )

        self._antennaControlClient = self.create_client (
            AntennaControlService,
            "/gs_tower_control/service"
        )
    
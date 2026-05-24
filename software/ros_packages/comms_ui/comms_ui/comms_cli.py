import comms_ui.console_ui as c
import comms_ui.console_ui_elements as ce
import rclpy
import time
import comms_ui.comms_ui_ros_node as rn
from gs_tower_control.control_script import AntennaControlMode, StatusFlags, AntennaControlStatus

console = c.ConsoleUI(exit)

titleFg = c.Color(255,255,255)
primaryFg = c.Color(200,200,200)
errorFg = c.Color(255,0,0)
primaryBg = c.Color(0,0,128)

node: rn.CommsUINode

commandResult = ce.StringField(
    console,
    0,
    console.get_size()[1] - 2,
    fg = c.Color(0, 0, 0),
    bg = c.Color(200, 200, 200)
)

statusTable = ce.TableField (
    console,
    1,
    1,
    {},
    "Contoller Status",
    None,
    None,
    primaryFg,
    primaryBg,
    titleFg
)

errorList = ce.ListField(
    console,
    60,
    1,
    None,
    "Active Errors",
    errorFg,
    primaryBg,
    titleFg
)

def handle_command(command: str) -> None:
    if command is None: 
        commandResult.set("Please enter a command")
        return
    tokens = command.split(' ')
    if tokens[0] == "manual":
        result = node.set_mode(AntennaControlMode.MANUAL_CONTROL)
        if not result:
            commandResult.set("Failed to enter manual control mode")
            return 
        else:
            commandResult.set("Entering manual control mode")
            return
    elif tokens[0] == "automatic":
        result = node.set_mode(AntennaControlMode.AUTOMATIC_CONTROL)
        if not result:
            commandResult.set("Failed to enter automatic control mode")
            return
        else:
            commandResult.set("Entering automatic control mode")
            return
    elif tokens[0] == "disable":
        result = node.set_mode(AntennaControlMode.DISABLED)
        if not result:
            commandResult.set("Failed to disable control")
            return
        else:
            commandResult.set("Disabling control")
            return
    elif tokens[0] == "home":
        result = node.set_mode(AntennaControlMode.HOMING)
        if not result:
            commandResult.set("Failed to begin homing sequence")
            return
        else:
            commandResult.set("Performing homing sequence")
            return
    elif tokens[0] == "pos":
        pan = 0.0
        elev = 0.0
        if len(tokens) < 3:
            commandResult.set("Please enter pan and elevation angle")
            return
        try:
            pan = float(tokens[1])
            elev = float(tokens[2])
        except:
            commandResult.set("Angle parameters invalid")
            return
        
        node.publish_manual_control_input(pan, elev)
        commandResult.set("Published manual control input")
        return
    else:
        commandResult.set(f"Unknown command: \"{tokens[0]}\" ")
        return
            

commandInput = ce.InputField(
    console,
    0,
    console.get_size()[1] - 1,
    console.get_size()[0],
    onEnter=handle_command,
    fg = c.Color(255, 255, 255),
    bg = c.Color(0, 0, 0)
)


def draw_status():
    status = node.get_status()
    if status is not None:
        statusTable.update_row("Current Operating Mode:", str(AntennaControlMode(status.operating_mode).name))
        statusTable.update_row("Elevation Angle Deg:", str(status.current_elevation_deg))
        statusTable.update_row("Elevation Setpoint:", str(status.current_elevation_setpoint))
        statusTable.update_row("Pan Angle Deg:", str(status.current_pan_deg))
        statusTable.update_row("Pan Setpoint:", str(status.current_pan_setpoint))
    
        errorList.clear()

        for f in StatusFlags:
            if f.value & status.errors:
                errorList.add_item(f.name)
        
        #errorList.add_item(str(status.errors))
    else:
        errorList.clear()
        errorList.add_item("NO_STATUS")



def main(args=None):
    global node

    #create node and init ros
    rclpy.init(args=args)
    node = rn.CommsUINode()

    while True:
        rclpy.spin_once(node, timeout_sec=0)

        draw_status()

        if not commandInput.is_active():
            commandInput.activate()

        console.update(primaryFg, primaryBg)




if __name__ == "__main__":
    main()
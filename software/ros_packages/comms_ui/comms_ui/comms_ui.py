import edifice as ed
from typing import Optional
from PySide6 import QtCore

#import comms_ui.comms_ui_ros_backend as control
#from gs_tower_control.gs_tower_control.control_script import AntennaControlMode, StatusFlags

#controlNode: control.CommsUINode

@ed.component
def BlinkingLabel(self, text: str, blink: bool = False, style: Optional[dict] = None):
    visible, set_visible = ed.use_state(True)

    if blink:
        def setup():
            timer = QtCore.QTimer()
            timer.setInterval(500)
            timer.timeout.connect(lambda: set_visible(lambda current: not current))
            timer.start()

            def cleanup():
                timer.stop()

            return cleanup

        ed.use_effect(setup, ())

    render_text = text if (not blink or visible) else ""
    ed.Label(
        text=render_text,
        style={**(style or {}), "font-size": "14px", "font-weight": "bold"},
    )


@ed.component
def IndicatorDot(self, color: str, blink: bool = False):
    visible, set_visible = ed.use_state(True)

    if blink:
        def setup():
            timer = QtCore.QTimer()
            timer.setInterval(500)
            timer.timeout.connect(lambda: set_visible(lambda current: not current))
            timer.start()

            def cleanup():
                timer.stop()

            return cleanup

        ed.use_effect(setup, ())

    opacity = 1.0 if visible else 0.0
    ed.Label(
        text="",
        style={
            "width": "14px",
            "height": "14px",
            "min-width": "14px",
            "min-height": "14px",
            "border-radius": "7px",
            "background-color": color,
            "opacity": opacity,
        },
    )


@ed.component
def AxisReadout(
    self,
    axis_name: str,
    position: Optional[float] = None,
    setpoint: Optional[float] = None,
    has_error: bool = False,
    is_calibrated: bool = True,
):
    with ed.VBoxView(
        style={
            "padding": "16px",
            "border": "1px solid #CCCCCC",
            "border-radius": "12px",
            "min-width": "220px",
            "background-color": "#F7F7F7",
        }
    ):
        with ed.HBoxView(style={"align": "center"}):
            ed.Label(text=axis_name, style={"font-size": "16px", "font-weight": "bold"})
            if has_error:
                ed.Label(
                    text="⚠",
                    style={"color": "#FF0000", "font-size": "18px", "margin-left": "8px"},
                    tool_tip=f"{axis_name} axis error",
                )

        ed.Label(text="Position:", style={"font-size": "12px", "margin-top": "12px"})
        position_text = f"{position:.1f}°" if position is not None else "0°"
        BlinkingLabel(
            text=position_text,
            blink=(position is None or not is_calibrated),
            style={"margin-top": "4px"},
        )

        ed.Label(text="Setpoint:", style={"font-size": "12px", "margin-top": "14px"})
        setpoint_text = f"{setpoint:.1f}°" if setpoint is not None else "0°"
        BlinkingLabel(
            text=setpoint_text,
            blink=(setpoint is None or not is_calibrated),
            style={"margin-top": "4px"},
        )


@ed.component
def ModeIndicator(self, mode: str, is_active: bool, mode_unknown: bool = False):
    dot_color = "#FF0000" if mode_unknown else ("#00AA00" if is_active else "#888888")
    with ed.ButtonView(
        on_trigger=lambda _: None,
        style={
            "align": "center",
            "padding": "8px",
            "border": "1px solid #DDDDDD",
            "border-radius": "6px",
            "background-color": "#FFFFFF",
        }
    ):
        IndicatorDot(color=dot_color, blink=mode_unknown)
        ed.Label(text=mode, style={"font-size": "12px"})


@ed.component
def ModeSelector(self, active_mode: Optional[str] = None, mode_unknown: bool = False):
    modes = ["Disabled", "Homing", "Manual", "Automatic"]
    with ed.VBoxView(
        style={
            "padding": "14px",
            "border": "1px solid #CCCCCC",
            "border-radius": "12px",
            "background-color": "#F7F7F7",
            "min-width": "220px",
        }
    ):
        ed.Label(text="Mode:", style={"font-size": "14px", "font-weight": "bold", "margin-bottom": "10px"})
        for mode in modes:
            ModeIndicator(
                mode=mode,
                is_active=(mode == active_mode),
                mode_unknown=mode_unknown,
            )


@ed.component
def ArrowPad(self, manual_mode_active: bool = False, open_angle_dialog=None):
    with ed.VBoxView(
        style={
            "align": "center",
            "padding": "12px",
            "border": "1px solid #CCCCCC",
            "border-radius": "12px",
            "background-color": "#F7F7F7",
        }
    ):
        with ed.HBoxView(style={"align": "center"}):
            ed.Button(
                title="↑",
                on_click=lambda _: None,
                enabled=manual_mode_active,
                style={"width": "52px", "height": "52px"},
            )
        with ed.HBoxView(style={"align": "center"}):
            ed.Button(
                title="←",
                on_click=lambda _: None,
                enabled=manual_mode_active,
                style={"width": "52px", "height": "52px", "margin-right": "8px"},
            )
            ed.Button(
                title="⊙",
                on_click=open_angle_dialog,
                enabled=manual_mode_active,
                style={"width": "52px", "height": "52px"},
            )
            ed.Button(
                title="→",
                on_click=lambda _: None,
                enabled=manual_mode_active,
                style={"width": "52px", "height": "52px", "margin-left": "8px"},
            )
        with ed.HBoxView(style={"align": "center"}):
            ed.Button(
                title="↓",
                on_click=lambda _: None,
                enabled=manual_mode_active,
                style={"width": "52px", "height": "52px", "margin-top": "8px"},
            )


@ed.component
def AngleDialog(
    self,
    pan_value: str,
    elevation_value: str,
    on_pan_change,
    on_elevation_change,
    on_save,
    on_cancel,
):
    with ed.WindowPopView(
        title="Enter exact axis angles",
        _size_open=(360, 240),
        on_close=lambda _: on_cancel(),
    ):
        with ed.VBoxView(style={"padding": "16px", "align": "center"}):
            ed.Label(text="Pan angle (deg):", style={"font-size": "12px"})
            ed.TextInput(
                text=pan_value,
                placeholder_text="Pan degrees",
                on_change=on_pan_change,
            )
            ed.Label(text="Elevation angle (deg):", style={"font-size": "12px", "margin-top": "8px"})
            ed.TextInput(
                text=elevation_value,
                placeholder_text="Elevation degrees",
                on_change=on_elevation_change,
            )
            with ed.HBoxView(style={"align": "right"}):
                ed.Button(title="Cancel", on_click=lambda _: on_cancel())
                ed.Button(title="Save", on_click=lambda _: on_save())

#TODO export variables that need changed by external.
@ed.component
def AntennaController(self):
    pan_position, set_pan_position = ed.use_state(None)
    pan_setpoint, set_pan_setpoint = ed.use_state(None)
    pan_calibrated, set_pan_calibrated = ed.use_state(False)
    pan_error, set_pan_error = ed.use_state(False)

    elevation_position, set_elevation_position = ed.use_state(None)
    elevation_setpoint, set_elevation_setpoint = ed.use_state(None)
    elevation_calibrated, set_elevation_calibrated = ed.use_state(False)
    elevation_error, set_elevation_error = ed.use_state(False)

    active_mode, set_active_mode = ed.use_state(None)
    mode_unknown, set_mode_unknown = ed.use_state(True)
    system_errors, set_system_errors = ed.use_state([])

    show_angle_dialog, set_show_angle_dialog = ed.use_state(False)
    pan_input, set_pan_input = ed.use_state("0")
    elevation_input, set_elevation_input = ed.use_state("0")

    #async def updateState():
    #    status = controlNode.get_status()
#
    #    if status is None:
    #        #set no communication error here
    #        return
    #    
    #    set_pan_position(status.current_pan_deg)
    #    set_elevation_position(status.current_elevation_deg)
    #    set_pan_setpoint(status.current_pan_setpoint)
    #    set_elevation_setpoint(status.current_elevation_setpoint)
    #    set_pan_calibrated(bool(not (status.errors & StatusFlags.AXIS_UNCALIBRATED_PAN.value)))
    #    set_elevation_calibrated(bool(not (status.errors & StatusFlags.AXIS_UNCALIBRATED_ELEV.value)))
#
    #ed.use_async_call(updateState)

    def open_angle_dialog(_=None):
        set_pan_input(str(pan_position) if pan_position is not None else "0")
        set_elevation_input(str(elevation_position) if elevation_position is not None else "0")
        set_show_angle_dialog(True)

    def save_angles(_=None):
        try:
            set_pan_position(float(pan_input))
        except ValueError:
            pass
        try:
            set_elevation_position(float(elevation_input))
        except ValueError:
            pass
        set_show_angle_dialog(False)

    def cancel_dialog(_=None):
        set_show_angle_dialog(False)

    with ed.Window(title="Antenna Controller", _size_open=(920, 520)):
        with ed.VBoxView(style={"padding": "20px", "background-color": "#FFFFFF"}):
            with ed.HBoxView(style={"align": "center"}):
                ed.Label(text="Antenna Controller", style={"font-size": "28px", "font-weight": "bold"})
                if len(system_errors) > 0:
                    ed.Label(
                        text="⚠",
                        style={"font-size": "24px", "color": "#FF0000", "margin-left": "10px"},
                        tool_tip="\n".join(system_errors),
                    )

            with ed.HBoxView(style={"align": "center"}):
                AxisReadout(
                    axis_name="Pan",
                    position=pan_position,
                    setpoint=pan_setpoint,
                    has_error=pan_error,
                    is_calibrated=pan_calibrated,
                )
                ed.Label(text="", style={"width": "20px"})
                AxisReadout(
                    axis_name="Elevation",
                    position=elevation_position,
                    setpoint=elevation_setpoint,
                    has_error=elevation_error,
                    is_calibrated=elevation_calibrated,
                )
                ed.Label(text="", style={"width": "20px"})
                ModeSelector(active_mode=active_mode, mode_unknown=mode_unknown)

            with ed.HBoxView(style={"align": "center"}):
                ArrowPad(
                    manual_mode_active=(active_mode == "Manual"),
                    open_angle_dialog=open_angle_dialog,
                )

            if show_angle_dialog:
                AngleDialog(
                    pan_value=pan_input,
                    elevation_value=elevation_input,
                    on_pan_change=set_pan_input,
                    on_elevation_change=set_elevation_input,
                    on_save=save_angles,
                    on_cancel=cancel_dialog,
                )


def main():
    ed.App(AntennaController()).start()


if __name__ == "__main__":
    main()

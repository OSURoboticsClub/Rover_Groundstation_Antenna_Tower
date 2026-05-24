import sys

import comms_ui.console_ui as cui
import comms_ui.console_ui_elements as ui


class FakeConsole:
    def __init__(self):
        self.draws = []
        self._next_callback_id = 0

    def register_update_callback(self, callback):
        self._next_callback_id += 1
        return self._next_callback_id

    def draw_string(self, x, y, string, fg, bg, maxLen=sys.maxsize):
        self.draws.append((x, y, string, fg, bg, maxLen))


def test_table_field_uses_title_and_computed_widths():
    console = FakeConsole()
    field = ui.TableField(
        console,
        x=2,
        y=4,
        data={"Alpha": "One", "LongKey": "TwoLong"},
        title="Status",
    )

    field.update()

    assert console.draws[0] == (2, 4, "Status", field._fg, field._bg, sys.maxsize)
    assert console.draws[1] == (2, 5, "Alpha    One    ", field._fg, field._bg, 16)
    assert console.draws[2] == (2, 6, "LongKey  TwoLong", field._fg, field._bg, 16)


def test_table_field_respects_explicit_widths():
    console = FakeConsole()
    field = ui.TableField(
        console,
        x=0,
        y=0,
        data={"alpha": "value"},
        title="T",
        col1Width=3,
        col2Width=2,
    )

    field.update()

    assert console.draws[0] == (0, 0, "T", field._fg, field._bg, sys.maxsize)
    assert console.draws[1] == (0, 1, "alp  va", field._fg, field._bg, 7)


def test_table_field_update_row_recomputes_auto_widths():
    console = FakeConsole()
    field = ui.TableField(
        console,
        x=1,
        y=2,
        data={"Alpha": "One", "LongKey": "TwoLong"},
        title="Status",
    )

    field.update_row("Alpha", "LongerValue")
    field.update()

    assert console.draws[0] == (1, 2, "Status", field._fg, field._bg, sys.maxsize)
    assert console.draws[1] == (1, 3, "Alpha    LongerValue", field._fg, field._bg, 20)
    assert console.draws[2] == (1, 4, "LongKey  TwoLong    ", field._fg, field._bg, 20)


def test_table_field_remove_row_and_keep_rendering():
    console = FakeConsole()
    field = ui.TableField(
        console,
        x=0,
        y=0,
        data={"Alpha": "One", "LongKey": "TwoLong"},
        title="Status",
    )

    assert field.remove_row("Alpha") is True
    field.update()

    assert console.draws[0] == (0, 0, "Status", field._fg, field._bg, sys.maxsize)
    assert console.draws[1] == (0, 1, "LongKey  TwoLong", field._fg, field._bg, 16)


def test_table_field_supports_separate_title_colors():
    console = FakeConsole()
    title_fg = cui.Color(1, 2, 3)
    title_bg = cui.Color(4, 5, 6)

    field = ui.TableField(
        console,
        x=2,
        y=4,
        data={"Alpha": "One"},
        title="Status",
        title_fg=title_fg,
        title_bg=title_bg,
    )

    field.update()

    assert console.draws[0] == (2, 4, "Status", title_fg, title_bg, sys.maxsize)
    assert console.draws[1] == (2, 5, "Alpha  One", field._fg, field._bg, 10)

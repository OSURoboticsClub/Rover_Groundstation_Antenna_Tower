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


def test_list_field_renders_title_and_entries():
    console = FakeConsole()
    title_fg = cui.Color(1, 2, 3)
    title_bg = cui.Color(4, 5, 6)

    field = ui.ListField(
        console,
        x=1,
        y=2,
        data=["Alpha", "Beta"],
        title="Items",
        title_fg=title_fg,
        title_bg=title_bg,
    )

    field.update()

    assert console.draws[0] == (1, 2, "Items", title_fg, title_bg, sys.maxsize)
    assert console.draws[1] == (1, 3, "Alpha", field._fg, field._bg, sys.maxsize)
    assert console.draws[2] == (1, 4, "Beta", field._fg, field._bg, sys.maxsize)


def test_list_field_add_remove_and_clear_items():
    console = FakeConsole()
    field = ui.ListField(console, x=0, y=0, data=["Alpha"], title="Items")

    field.add_item("Beta")
    field.update()
    assert console.draws[-1] == (0, 2, "Beta", field._fg, field._bg, sys.maxsize)

    assert field.remove_item("Alpha") is True
    field.update()
    assert console.draws[-1] == (0, 1, "Beta", field._fg, field._bg, sys.maxsize)

    field.clear()
    assert field._data == []
    field.update()
    assert console.draws[-1] == (0, 0, "Items", field._fg, field._bg, sys.maxsize)

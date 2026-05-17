import math
import copy
import sys
import tty
import os
from typing import Any, Tuple
from typing import List
import typing

ESC = '\x1b'

class Color():
        r: int
        g: int
        b: int

        def __init__(self, r: int, g: int, b: int):
            if r in range(0, 256) and g in range(0, 256) and b in range(0, 256):
                self.r = r
                self.g = g
                self.b = b

        def __setattr__(self, name: str, value: Any) -> None:
            if type(value) is not int or value not in range(0, 256):
                raise(ValueError("Channel values must be between 0 and 255"))
            
            super().__setattr__(name, value)


class ConsoleUI:

    #TODO More testing
    # Cleanup on exit
    # Fix input text scrolling


    _sizeX: int
    _sizeY: int
    _color: bool
    _charsBuffer: List[List[str]]
    _colorsBuffer: List[List[Tuple[Color, Color]]]
    _exitCallback: typing.Callable
    _collectInput: bool
    _inputBuffer: str
    _lastInput: str
    _keyBinds: dict[str, typing.Callable]


    def __init__(self, exitCallback: typing.Callable, color: bool = True, sizeX: int = 80, sizeY: int = 24):
        self.resize(sizeX, sizeY)
        self._color = color
        self._exitCallback = exitCallback
        self._collectInput = False
        self._inputBuffer = ""
        self._lastInput = ""

        #initialize console
        #disable automatic line wrapping
        sys.stdout.write(ESC+"[=7l")
        #set console to raw, non-blocking mode
        tty.setraw(sys.stdin)
        os.set_blocking(sys.stdin.fileno(), False)

        #TODO get console size from esc sequence

    
    def _console_write(self, string: str):

        #This strikes me as a disgusting hack, but seems to be very effective.
        #It seems that either bytes are duplicated, or the console cant interpret
        #broken escape sequences when attempting to write in nonblocking mode using
        #the commented logic at the bottom.

        os.set_blocking(sys.stdin.fileno(), True)

        sys.stdout.write(string)

        os.set_blocking(sys.stdin.fileno(), False)

        return

        #buffer = string.encode()
        #bytes = len(buffer)
        #while bytes > 0:
        #    try:
        #        bytes -= os.write(sys.stdout.fileno(), string.encode())
        #    except BlockingIOError as e:
        #        if e.errno == 11:
        #            continue
        #        else:
        #            raise


    def _set_cursor_visibility(self, visible: bool):
        if visible:
            self._console_write(ESC+"[?25h")
        else:
            self._console_write(ESC+"[?25l")


    def _set_cursor_position(self, x: int, y: int):
        self._console_write(ESC+f"[{y+1};{x+1}H")


    def _color_str(self, foreground: Color, background: Color) -> str:
        if self._color:
            return ESC+f"[38;2;{foreground.r};{foreground.g};{foreground.b}m" + ESC+f"[48;2;{background.r};{background.g};{background.b}m"
        else:
            return ""


    def _set_color(self, foreground: Color, background: Color):
        if self._color:
            self._console_write(self._color_str(foreground, background))
            sys.stdout.flush()


    def resize(self, sizeX: int, sizeY: int):

        if sizeX < 0 or sizeY < 0:
            raise ValueError("Size must be positive.")

        self._sizeX = sizeX
        self._sizeY = sizeY
        self._charsBuffer = list()
        self._colorsBuffer = list()
        for x in range(0, sizeX):
            self._charsBuffer.append(list())
            self._colorsBuffer.append(list())
            for y in range(0, sizeY):
                self._charsBuffer[x].append('')
                self._colorsBuffer[x].append((Color(255,255,255), Color(0,0,0)))

    
    def fill_buffer(self, char: str, fg: Color, bg: Color):
        if len(char) > 1:
            raise ValueError("Char must be single character.")
        
        for x in range(0, self._sizeX):
            for y in range(0, self._sizeY):
                self._charsBuffer[x][y] = char
                self._colorsBuffer[x][y] = (fg, bg)

    
    def draw_char(self, x: int, y: int, char: str, fg: Color, bg: Color):
        if x < 0 or y < 0 or x >= self._sizeX or y >= self._sizeY:
            return
        
        self._charsBuffer[x][y] = char
        self._colorsBuffer[x][y] = (fg, bg)

    
    def draw_string(self, x: int, y: int, string: str, fg: Color, bg: Color, maxLen: int = sys.maxsize):
        for i in range(len(string) if len(string) < maxLen else maxLen):
            self.draw_char(x + i, y, string[i], fg, bg)

    
    def draw_table(
            self,
            x: int,
            y: int,
            table: list[list[tuple[str, Color, Color]]],
            title: typing.Optional[tuple[str, Color, Color]],
            colWidth: int = 0,
            fixedColWidth: bool = False,
    ):
        pass


    def write_buffer(self):
        frame = ""

        self._set_cursor_position(0, 0)
        self._set_cursor_visibility(False)

        prevColorStr = None

        for y in range(self._sizeY):
            for x in range(self._sizeX):
                colorStr = self._color_str(self._colorsBuffer[x][y][0], self._colorsBuffer[x][y][1])
                if colorStr != prevColorStr:
                    frame += colorStr
                    prevColorStr = colorStr
                frame += self._charsBuffer[x][y]
                if self._charsBuffer[x][y] == "":
                    frame += " "
            frame += "\r\n"

        self._console_write(frame)
        sys.stdout.flush()

    #bind callback to a keyboard input. 
    def bind_key(self, key: int, callback: typing.Callable):
        pass


    def get_input(self, x: int, y: int, fg: Color, bg: Color, boxLength: typing.Optional[int] = None):
        self._inputX = x
        self._inputY = y
        if boxLength is None or x + boxLength >= self._sizeX:
            self._inputBoxLength = self._sizeX - x
        else: 
            self._inputBoxLength = boxLength
        self._collectInput = True
        self._inputColor = (fg, bg)


    def get_last_input(self) -> str:
        return self._lastInput


    def handle_input(self):

        #return

        def run_keybind(key: str):
            func = self._keyBinds.get(key)
            if func is not None:
                func()

        escSeq = False
        
        input = ""

        try:
            input = sys.stdin.read()
        except Exception:
            c = ""

        for c in input:
            if escSeq:
                if ord(c) >= 97:
                    escSeq = False
                    continue
                #TODO handle escape sequences here
            if c == ESC:
                escSeq = True
            #handle CTRL+C
            elif ord(c) == 3:
                self._exitCallback()
            #handle backspace and delete
            elif ord(c) == 8 or ord(c) == 127:
                #if currently collecting input, remove most last char from input buffer
                if self._collectInput:
                    if self._inputBuffer != "":
                        self._inputBuffer  = self._inputBuffer[0:-1]
            #handle enter
            elif ord(c) == 10 or ord(c) == 13:
                #if collecting input, finish and save buffer
                if self._collectInput:
                    self._collectInput = False
                    self._lastInput = self._inputBuffer
                #otherwise run a keybind if one exists
                else:
                    run_keybind(c)
            #handle printable characters
            elif ord(c) in range(32, 127):
                if self._collectInput:
                    self._inputBuffer += c
                else:
                    run_keybind(c)
            else:
                run_keybind(c)

        #echo user input if necessary
        if self._collectInput:
            startIndex = 0
            if len(self._inputBuffer) > self._inputBoxLength:
                index = len(self._inputBuffer) - self._inputBoxLength
            
            for i in range(len(self._inputBuffer) - startIndex):
                self.draw_char(self._inputX + i, self._inputY, self._inputBuffer[i], self._inputColor[0], self._inputColor[1])

            #self._set_cursor_position(
            #    self._inputX + (self._inputBoxLength - 1 if self._inputBoxLength >= len(self._inputBuffer) else len(self._inputBuffer)), 
            #    self._inputY)
            #self._set_cursor_visibility(True)



    



    

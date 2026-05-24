#import comms_ui.console_graphics as cg
import console_ui as cui
import console_ui_elements as ui
import time

def main():

    console = cui.ConsoleUI(exit, automaticSize=True)

    console.get_input(0, 1, cui.Color(255,0,0), cui.Color(0, 0, 128))

    while(True):
        for x in range(console.get_size()[0]):
            console.fill_buffer("", cui.Color(0,0,0), cui.Color(0,0,255))
            console.draw_char(x, 10, '#', cui.Color(255,255,255), cui.Color(0,0,128))
            console.handle_input()
            console.write_buffer()
            #time.sleep(0.05)


if __name__ == "__main__":
    main()
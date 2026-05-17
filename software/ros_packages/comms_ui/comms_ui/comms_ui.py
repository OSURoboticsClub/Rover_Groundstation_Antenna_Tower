#import comms_ui.console_graphics as cg
import console_ui as cui
import time

def main():

    console = cui.ConsoleUI(exit)

    console.get_input(0, 0, cui.Color(255,0,0), cui.Color(0, 0, 128))

    while(True):
        for x in range(80):
            console.fill_buffer("", cui.Color(0,0,0), cui.Color(0,0,0))
            for i in range(80):
                for j in range(0, 24, 2):
                    console.draw_char(i, j, ' ', cui.Color(0,0,0), cui.Color(2*i, 2*i, 2*i))
                    console.draw_char(i, j + 1, ' ', cui.Color(0,0,0), cui.Color(160 - i*2, 160 - 2*i, 160 - 2*i))
            console.draw_char(x, 10, '#', cui.Color(255,255,255), cui.Color(0,0,128))
            console.handle_input()
            console.write_buffer()
            time.sleep(0.05)


if __name__ == "__main__":
    main()
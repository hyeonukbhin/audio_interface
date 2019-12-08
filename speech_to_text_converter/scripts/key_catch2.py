#!/usr/bin/python3
# -*- coding: utf-8 -*-

from pynput import mouse
import threading

from time import sleep

key_state = "released"
def on_click(x, y, button, pressed):
    global key_state
    # print("Pressed")
    key_state = "pressed"
    # if button == mouse.Button.left:
        # return False

    if not pressed:
        # print("Released")
        key_state = "released"
        # return False


def main():
    global key_state
    listener = mouse.Listener(on_click=on_click)
    listener.start()

    while True:
        print("111")
        print(key_state)
        # with mouse.Listener(on_click=on_click) as listener:
        #     listener.join()
            # listener.wait()
        # listener.stop()


        print("222")
        sleep(1)




# def main():
#     # thread2 = threading.Thread(target=tt, args=())
#     # thread2.start()
#     while True:
#
#     listener = mouse.Listener(on_click=on_click)
#     listener.start()


    # with mouse.Listener(on_press=on_click) as listener:
    #     listener.join()



main()
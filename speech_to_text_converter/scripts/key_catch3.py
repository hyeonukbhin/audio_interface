#!/usr/bin/python3
# -*- coding: utf-8 -*-
# !/usr/bin/env python
import time
# from pynput.keyboard import Key, Listener
from pynput.mouse import Listener
import threading


def main():
    thread2 = threading.Thread(target=takeScreenshot, args=())
    thread2.start()

    listener = Listener(on_click=on_click)
    listener.start()
    # with Listener(on_press=on_click) as listener:
    #     listener.join()


def getKey(key):
    print(key)
    key = fixKey(key)
    file = open('log.txt', 'a')
    file.write(key.replace('\'', '') + '')
    file.close()

def fixKey(key):
    key = str(key)
    if key == 'Key.space':
        return ' '
    elif key == 'Key.enter':
        return '\n'
    return key

def on_click(x, y, button, pressed):
    # print('{0} at {1}'.format('Pressed' if pressed else 'Released',(x, y)))

    # print("Pressed")
    # return False
    print("click")
    # if button == mouse.Button.left:
        # return False
    #
    # if not pressed:
    #     print("Released")
    #     # Stop listener
    #     return False

def takeScreenshot():
    # run contineous and take screenshot every 15 seconds
    while True:
        print('taking screenshot')
        now = time.strftime("%d-%m-%Y" + ' ' + "%H-%M-%S")
        print(now)
        time.sleep(15)


main()
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np   # for zeros
import cv2 as cv



mouse_event_types = { 0:"EVENT_MOUSEMOVE", 1:"EVENT_LBUTTONDOWN", 2:"EVENT_RBUTTONDOWN", 3:"EVENT_MBUTTONDOWN",
                 4:"EVENT_LBUTTONUP", 5:"EVENT_RBUTTONUP", 6:"EVENT_MBUTTONUP",
                 7:"EVENT_LBUTTONDBLCLK", 8:"EVENT_RBUTTONDBLCLK", 9:"EVENT_MBUTTONDBLCLK",
                 10:"EVENT_MOUSEWHEEL", 11:"EVENT_MOUSEHWHEEL"}

mouse_event_flags = { 0:"None", 1:"EVENT_FLAG_LBUTTON", 2:"EVENT_FLAG_RBUTTON", 4:"EVENT_FLAG_MBUTTON",
                8:"EVENT_FLAG_CTRLKEY", 9:"EVENT_FLAG_CTRLKEY + EVENT_FLAG_LBUTTON",
                10:"EVENT_FLAG_CTRLKEY + EVENT_FLAG_RBUTTON", 11:"EVENT_FLAG_CTRLKEY + EVENT_FLAG_MBUTTON",

                16:"EVENT_FLAG_SHIFTKEY", 17:"EVENT_FLAG_SHIFTKEY + EVENT_FLAG_LBUTTON",
                18:"EVENT_FLAG_SHIFTLKEY + EVENT_FLAG_RBUTTON", 19:"EVENT_FLAG_SHIFTKEY + EVENT_FLAG_MBUTTON",

                32:"EVENT_FLAG_ALTKEY", 33:"EVENT_FLAG_ALTKEY + EVENT_FLAG_LBUTTON",
                34:"EVENT_FLAG_ALTKEY + EVENT_FLAG_RBUTTON", 35:"EVENT_FLAG_ALTKEY + EVENT_FLAG_MBUTTON"}



# 1. 마우스 이벤트 발생시 호출될 함수를 정의합니다.
def mouse_callback(event, x, y, flags, param):
    print("마우스 이벤트 발생")
    print(flags)
    print( '( '+ str(x) + ' ' + str(y), ')' + ' ' + mouse_event_types[event])

    if event == 10:
        if flags > 0:
            print("forward scrolling")
        else:
            print("backward scrolling")
    elif event == 11:
        if flags > 0:
            print("right scrolling")
        else:
            print("left scrolling")
    else:
        print( mouse_event_flags[flags])



img = np.zeros((512, 512, 3), np.uint8)
cv.namedWindow('image')  # 2. 마우스 이벤트를 감지할 윈도우를 생성합니다.


# 3. 이름이 image인 윈도우에서 마우스 이벤트가 발생하면 mouse_callback 함수가 호출되게 됩니다.
cv.setMouseCallback('', mouse_callback)


cv.imshow('image',img)
cv.waitKey(0)

cv.destroyAllWindows()
# UnderWaterDogs

import pymurapi as mur
import time
import cv2 as cv
import math

auv = mur.mur_init()
degree = 0
colors = {'blue': ((130, 0, 0), (180, 255, 230)),
          'yellow': ((0, 0, 0), (30, 255, 205)),
          'green': ((60, 0, 0), (91, 255, 255))}
x_center, y_center, x, y = 999, 999, 999, 999


def clamp(v, min, max):
    if v > max:
        return max
    if v < min:
        return min
    return v


class PD(object):
    _kp = 0.0
    _kd = 0.0
    _prev_error = 0.0
    _timestamp = 0

    def __init__(self):
        pass

    def set_p(self, value):
        self._kp = value

    def set_d(self, value):
        self._kd = value

    def process(self, error):
        timestamp = int(round(time.time() * 1000))
        out = self._kp * error + self._kd / (timestamp - self._timestamp) * (error - self._prev_error)
        self._timestamp = timestamp
        self._prev_error = error
        return out


def keep_depth(depth, P, D):
    try:
        error = auv.get_depth() - depth
        out = keep_depth.reg.process(error)
        out = clamp(out, -100, 100)
        auv.set_motor_power(2, out)
        auv.set_motor_power(3, out)
    except:
        keep_depth.reg = PD()
        keep_depth.reg.set_p(P)
        keep_depth.reg.set_d(D)


def keep_yaw(yaw, power, P, D):
    def to_180(angle):
        if angle > 180.0:
            return angle - 360
        if angle < -180.0:
            return angle + 360
        return angle

    try:
        error = auv.get_yaw() - yaw
        error = to_180(error)
        out = keep_yaw.reg.process(error)
        out = clamp(out, -100, 100)
        auv.set_motor_power(0, clamp((power - out), -100, 100))
        auv.set_motor_power(1, clamp((power + out), -100, 100))
    except:
        keep_yaw.reg = PD()
        keep_yaw.reg.set_p(P)
        keep_yaw.reg.set_d(D)


def get_cont(img, color):
    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(img_hsv, color[0], color[1])
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    return contours


def draw_cont(img, contour):
    global x_center, y_center, x, y
    if cv.contourArea(contour) < 100:
        return
    cv.drawContours(img, [contour], 0, (255, 255, 255), 2)
    moments = cv.moments(contour)
    xm1 = moments['m10']
    xm2 = moments['m00']
    ym1 = moments['m01']
    ym2 = moments['m00']
    x = int(xm1 / xm2)
    y = int(ym1 / ym2)
    x_center = x - (320 / 2)
    y_center = y - (240 / 2)
    cv.circle(img, (x, y), 3, (0, 0, 255), -1)
    return x_center, y_center, x, y


def get_color(color):
    img = auv.get_image_bottom()
    cont_img = img.copy()
    for _ in colors:
        contours = get_cont(img, colors[color])
        if not contours:
            continue
        for cnt in contours:
            draw_cont(cont_img, cnt)
    cv.imshow("gen", img)
    cv.imshow("cont", cont_img)
    cv.waitKey(1)


def turn(degres, depth):
    timing = time.time()
    while True:
        get_color('green')
        keep_depth(depth, 70, 5)
        keep_yaw(degres, 0, 0.8, 0.5)
        if time.time() - timing > 8:
            timing = time.time()
            break


def go(degres, power, time_, depth):
    timing = time.time()
    while True:
        get_color('green')
        keep_depth(depth, 50, 7)
        keep_yaw(degres, power, 0.8, 0.5)
        if time.time() - timing > time_:
            timing = time.time()
            break


def depthing(depth, time_):
    timing = time.time()
    while True:
        get_color('green')
        keep_yaw(degree, 0, 0.8, 0.5)
        keep_depth(depth, 40, 6)
        if time.time() - timing > time_:
            timing = time.time()
            break


def centralize(color):
    x_center = x - (320 / 2)
    y_center = y - (240 / 2) + 22
    try:
        get_color(color)
        lenght = math.sqrt(x_center ** 2 + y_center ** 2)
        if lenght < 2.0:
            return True
        outForward = centralize.regForward.process(y_center)
        outForward = clamp(outForward, -20, 20)
        outSide = centralize.regSide.process(x_center)
        outSide = clamp(outSide, -20, 20)
        auv.set_motor_power(0, -outForward)
        auv.set_motor_power(1, -outForward)
        auv.set_motor_power(4, -outSide)
    except:
        centralize.regForward = PD()
        centralize.regForward.set_p(0.3)
        centralize.regForward.set_d(0.1)

        centralize.regSide = PD()
        centralize.regSide.set_p(0.2)
        centralize.regSide.set_d(0.25)
    return False


def stopCenterlize(color):
    centralize(color)
    if centralize(color):
        auv.set_motor_power(0, 0)
        auv.set_motor_power(1, 0)
        auv.set_motor_power(4, 0)
        return True

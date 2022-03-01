import pymurapi as mur
import time
import cv2 as cv
import math

activity = 'Grab'
auv = mur.mur_init()
x_center, y_center = 999, 999
averageN, maxDrop = 0, 0
x, y = 999, 999
maxN, minN = 0, 0
fC, degree = 0, 0
kolN = 0
sumDropNow = 0
ord = []
drop = []
endIs = 0
colors = {'blue': ((130, 0, 0), (180, 255, 230)),
          'yellow': ((0, 0, 0), (30, 255, 205)),
          'green': ((60, 0, 0), (91, 255, 255))}


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
    for name in colors:
        contours = get_cont(img, colors[color])
        if not contours:
            continue
        for cnt in contours:
            draw_cont(cont_img, cnt)
    cv.imshow("gen", img)
    cv.imshow("cont", cont_img)
    cv.waitKey(1)


def checking():
    global fC
    fC = 0
    img = auv.get_image_bottom()
    while True:
        keep_depth(2.3, 50, 7)
        keep_yaw(0, 0, 0.8, 0.5)
        get_color('blue')
        contours = get_cont(img, colors['blue'])
        for c in contours:
            if cv.contourArea(c) > 200:
                fC += 1
        ord.append(fC)
        break
    keep_yaw(0, 50, 0.8, 0.5)
    print(ord)
    time.sleep(1.5)


def turn(degree, depth):
    timing = time.time()
    while True:
        get_color('green')
        keep_depth(depth, 70, 5)
        keep_yaw(degree, 0, 0.8, 0.5)
        if time.time() - timing > 8:
            timing = time.time()
            break


def go(degree, power, time_, depth):
    timing = time.time()
    while True:
        get_color('green')
        keep_depth(depth, 50, 7)
        keep_yaw(degree, power, 0.8, 0.5)
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


def plan():
    global maxN, minN
    if max(max(ord[0], ord[1]), ord[2]) == ord[2]:
        maxN = 3
    elif max(max(ord[0], ord[1]), ord[2]) == ord[1]:
        maxN = 2
    elif max(max(ord[0], ord[1]), ord[2]) == ord[0]:
        maxN = 1
    if min(min(ord[0], ord[1]), ord[2]) == ord[2]:
        minN = 3
    elif min(min(ord[0], ord[1]), ord[2]) == ord[1]:
        minN = 2
    elif min(min(ord[0], ord[1]), ord[2]) == ord[0]:
        minN = 1


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


def checkOrd():
    plan()
    global activity, sumDropNow, endIs
    prev = 0
    for i in range(len(ord)):
        if averageN == ord[i]:
            endIs += 1
        sumDropNow = prev + clamp((averageN - ord[i]), 0, 999)
        prev = sumDropNow
    if sumDropNow > maxDrop:
        activity = 'Grab'
        print('Grab')
    elif endIs == 3:
        while True:
            depthing(0, 1)
    else:
        activity = 'Drop'
        print('Drop')


depthing(2.45, 5.5)
print("------------SCAN------------")
while True:
    keep_yaw(0, 90, 0.8, 0.5)
    keep_depth(2.6, 50, 4)
    get_color('green')
    if 10 >= x_center >= -10 and 10 >= y_center >= -10:
        checking()
    if len(ord) >= 3:
        break
print("------------GRAB------------")
for i in range(len(ord)):
    kolN += ord[i]
averageN = math.ceil(kolN / 3)
maxDrop = (averageN * 3) - kolN
print("AllCount:", kolN)
print("AverageCount:", averageN)
print("MaxDropCount:", maxDrop)
go(0, -100, 0.4, 2.8)
nowPoint = 4
while True:
    checkOrd()
    plan()
    if nowPoint < maxN:
        if degree != 0:
            degree = 0
            turn(0, 2.9)
        go(0, 70, 1.5, 3)
    else:
        if degree != 180:
            degree = 180
            turn(180, 2.9)
        go(180, 70, 1.5, 3)
    print("Go To", maxN)
    print('N', nowPoint)
    while True:
        keep_yaw(degree, 80, 0.8, 0.5)
        keep_depth(3.1, 70, 5)
        get_color('green')
        if 10 >= x_center >= -10 and 10 >= y_center >= -10:
            if nowPoint - maxN > 1 or nowPoint - maxN < -1:
                go(degree, 80, 1.5, 3)
            if degree == 180:
                nowPoint -= 1
            else:
                nowPoint += 1
        if nowPoint == maxN:
            break
    go(degree, -100, 2.3, 3.1)
    go(degree, 0, 1, 3.1)
    auv.open_grabber()
    print('=========Down============')
    timing = time.time()
    while True:
        centralize('blue')
        auv.set_motor_power(2, 0)
        auv.set_motor_power(3, 0)
        if time.time() - timing > 23:
            auv.set_motor_power(0, 0)
            auv.set_motor_power(1, 0)
            auv.set_motor_power(4, 0)
            break
    while True:
        get_color('blue')
        depthing(3.75, 4)
        auv.close_grabber()
        time.sleep(1)
        break
    print('Go To', minN)
    print('N', nowPoint)
    depthing(2.9, 5)
    timing = time.time()
    while True:
        centralize('green')
        keep_depth(2.7, 50, 7)
        if stopCenterlize('green') or time.time() - timing > 15:
            break
    if nowPoint < minN:
        degree = 0
        turn(0, 2.9)
        go(0, 50, 1.8, 2.7)
    else:
        degree = 180
        turn(180, 2.9)
        go(180, 50, 1.8, 2.7)
    timing = time.time()
    while True:
        keep_depth(2.7, 70, 5)
        keep_yaw(degree, 0, 0.8, 0.5)
        if stopCenterlize('green') or time.time() - timing > 2:
            auv.set_motor_power(4, 0)
            break
    while True:
        keep_yaw(degree, 80, 0.8, 0.5)
        keep_depth(2.7, 70, 5)
        get_color('green')
        if 12 >= x_center >= -12 and 12 >= y_center >= -12:
            if nowPoint - minN > 1 or nowPoint - minN < -1:
                print('PB')
                go(degree, 80, 1.2, 3)
            if degree == 180:
                nowPoint -= 1
            else:
                nowPoint += 1
        if nowPoint == minN:
            go(degree, -10, 0.3, 3)
            break
    go(degree, -100, 1.5, 3)
    go(degree, 0, 0.2, 3)
    auv.open_grabber()
    time.sleep(1)
    ord[minN - 1] += 1
    ord[maxN - 1] -= 1
    print(ord)
    for i in range(len(ord)):
        if ord[i] != averageN:
            checkOrd()
            break
        elif i == 2:
            while True:
                keep_depth(0, 999, 999)
    checkOrd()
    if activity == 'Drop':
        break
    plan()
    print('Go To', maxN)
    if nowPoint < maxN:
        if degree != 0:
            degree = 0
            turn(0, 2.9)
        go(0, 50, 1.6, 2.7)
    else:
        if degree != 180:
            degree = 180
            turn(180, 2.9)
        go(180, 50, 1.6, 2.7)
    timing = time.time()
    while True:
        keep_depth(2.7, 70, 5)
        keep_yaw(degree, 0, 0.8, 0.5)
        if stopCenterlize('yellow') or time.time() - timing > 3:
            auv.set_motor_power(4, 0)
            break
    checkOrd()
    if activity == 'Drop':
        break
    print('N', nowPoint)
plan()
if nowPoint == minN:
    maxDrop -= 1
    ord[minN - 1] += 1
    auv.drop()
while True:
    if maxDrop == 0:
        while True:
            depthing(0, 1)
    timing = time.time()
    while True:
        keep_depth(2.7, 70, 5)
        keep_yaw(degree, 0, 0.8, 0.5)
        if stopCenterlize('green') or time.time() - timing > 6:
            auv.set_motor_power(4, 0)
            break
    plan()
    print('Go To', minN)
    if nowPoint < minN:
        if degree != 0:
            degree = 0
            turn(0, 2.9)
        go(0, 50, 1.6, 2.7)
    else:
        if degree != 180:
            degree = 180
            turn(180, 2.9)
        go(180, 50, 1.6, 2.7)
    while True:
        if nowPoint == minN:
            go(degree, -10, 0.6, 3)
            break
        keep_yaw(degree, 80, 0.8, 0.5)
        keep_depth(2.7, 70, 5)
        get_color('green')
        if 13 >= x_center >= -13 and 13 >= y_center >= -13:
            if nowPoint - minN > 1 or nowPoint - minN < -1:
                print('PB')
                go(degree, 80, 1.2, 3)
            if degree == 180:
                nowPoint -= 1
            else:
                nowPoint += 1
    go(degree, -100, 1.6, 3)
    go(degree, 5, 0.2, 3)
    auv.drop()
    ord[minN - 1] += 1
    maxDrop -= 1
    if maxDrop == 0:
        while True:
            depthing(0, 1)
    checkOrd()
    if nowPoint < maxN:
        if degree != 0:
            degree = 0
            turn(0, 2.9)
        go(0, 50, 1.6, 2.7)
    else:
        if degree != 180:
            degree = 180
            turn(180, 2.9)
        go(180, 50, 1.6, 2.7)
    timing = time.time()
    while True:
        keep_depth(2.7, 70, 5)
        keep_yaw(degree, 0, 0.8, 0.5)
        if stopCenterlize('yellow') or time.time() - timing > 3:
            auv.set_motor_power(4, 0)
            break

import pymurapi as mur
import time
import cv2 as cv
import numpy as np
auv = mur.mur_init()

colors = {'blue': ((124,0,0), (140,255,230)),
          'yellow': ((0,0,0), (30,255,205)),
          'green': ((60,0,0), (91,255,255))}

def clamp_to_360(angle):
    if angle < 0.0:
        return angle + 360.0
    if angle > 360.0:
        return angle - 360.0
    return angle

# Перевод угла из 0 <=> 360 в -180 <=> 180
def to_180(angle):
    if angle > 180.0:
        return angle - 360.0
    return angle

# Преобразовать v в промежуток между min max
def clamp(v, min, max):
	if v < min:
		return min
	if v > max:
		return max
	return v

# Функция удержания курса
def keep_yaw(yaw_to_set, power):
    current_yaw = auv.get_yaw()
    er = clamp_to_360(yaw_to_set - current_yaw)
    er = to_180(er)
    res = er * -0.8
    auv.set_motor_power(0, clamp(int(power - res), -100, 100))
    auv.set_motor_power(1, clamp(int(power + res), -100, 100))


def keep_depth(depth_to_set):
    power = 30 * (auv.get_depth() - depth_to_set)
    auv.set_motor_power(2, clamp(int(power), -100, 100))
    auv.set_motor_power(3, clamp(int(power), -100, 100))
   
def get_cont(img, color):
    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(img_hsv, color[0], color[1])
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    return contours
    
def draw_cont(img, contour, name):
    if cv.contourArea(contour) < 250:
        return
    cv.drawContours(img, [contour], 0, (255,255,255), 2)
                
    moments = cv.moments(contour)
    xm1 = moments['m10']
    xm2 = moments['m00']
    ym1 = moments['m01']
    ym2 = moments['m00']
    x = int(xm1/xm2)
    y = int(ym1/ym2)
    cv.circle(img, (x,y), 3, (0,0,255), -1)

def get_color():
    img = auv.get_image_bottom()
    cont_img = img.copy()
    for name in colors:
        contours = get_cont(img, colors[name])
        if not contours:
            continue
        for cnt in contours:
            draw_cont(cont_img, cnt, name)
    cv.imshow("gen", img)
    cv.imshow("cont", cont_img)
    cv.waitKey(1)
    
def checkZone():
    isCheck = True
    while isCheck != False:
        get_color()
        keep_depth(2.3)
        keep_yaw(0, 20)
        
while True:
    checkZone()














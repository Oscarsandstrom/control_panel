#!/usr/bin/python

import struct
import rospy
import pygame
import math
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int8MultiArray

# Window variables
WIDTH = 1600
HEIGHT = 1000
FRAMERATE = 60


# Speed variables
speed_reference = 0
steering_reference = 0

# Control variables
coll_detect = False
coll_override = 0

emergency_state = False
emergency_prev = False

handBreak_state = True
handBreak_prev = False

beacon_state = False
beacon_prev = False


# Pygame
pygame.init()
s = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Control Panel")
clock = pygame.time.Clock()
clock.tick(FRAMERATE)



def joyCallback(data):
    #Axes
    global steering_reference
    global speed_reference

    steering_reference = data.axes[0]
    speed_reference = (-data.axes[5] + data.axes[2])

    #Buttons
    global coll_override
    global emergency_prev
    global emergency_state
    global handBreak_state
    global handBreak_prev
    global beacon_state
    global beacon_prev

    # B Button    
    coll_override = data.buttons[1]

    # A Button
    emergency_state, emergency_prev = toggle(data.buttons[0], emergency_state, emergency_prev)

    # Start Button
    handBreak_state, handBreak_prev = toggle(data.buttons[7], handBreak_state, handBreak_prev)

    # Y Button
    beacon_state, beacon_prev = toggle(data.buttons[3], beacon_state, beacon_prev)

def toggle(val, state, prev):
    if (val == 1) and (not prev):
	state = not state
    prev = (val == 1)
    return state, prev

def joyInputs():
 
    rospy.Subscriber('joy', Joy, joyCallback)


def stopCallback(data):
    global coll_detect
    
    coll_detect = data


def collDetect():
    
    rospy.Subscriber('lidar_stop', Bool, stopCallback)

def drawMeters(s, speed_reference, steering_reference):
    color = (255, 255, 255)

    speed_color = (0, 0, 255)    
    steering_color = (0, 0, 255)

    rh = 500
    rw = 150
    rgap = 200

    rx = WIDTH - (2 * rw + 2 * (rgap - rw))
    ry = (HEIGHT - rh)/2

    if steering_reference != 0:
	pos2 = ((rx - 1 * rh/3 + 16 + rh/2) + 250 * -steering_reference, ry + rh, 10, rw - 2)
	pygame.draw.rect(s, steering_color, pos2, 0)
    
    if speed_reference != 0:
        pos = (rx + 1 * rgap + 1, (ry - rw + rh/2) + 125 * -speed_reference, rw - 2, 10)
        pygame.draw.rect(s, speed_color, pos, 0)


    pos = (rx + 1 * rgap, ry - rw, rw, rh + 12)
    pos2 = (rx - 1 * rh/3 + 16, ry + rh, rh + 12, rw)
    pygame.draw.rect(s, color, pos, 2)
    pygame.draw.rect(s, color, pos2, 2)
  
    
def drawBoxes(s, collision, emergency):
    color = (255, 255, 255)
    reset_color = (0 ,0 ,0)

    stop_color = (255, 0, 0)
    override_color = (11, 102, 35)
    beacon_color = (255, 165, 0)

    tmargin = 100
    smargin = 250
    bmargin = 170

    bside = 150


    #Collision box
    if coll_detect:
	pos = (smargin, tmargin, bside, bside)
        pygame.draw.rect(s, stop_color, pos, 0)

    if  coll_override == 1:
	pos = (smargin, tmargin, bside, bside)
        pygame.draw.rect(s, override_color, pos, 0)

    pos = (smargin, tmargin, bside, bside)
    pygame.draw.rect(s, color, pos, 2)

    #Emergency break box
    if emergency_state:
	pos = (smargin + bmargin, tmargin, bside, bside)
	pygame.draw.rect(s, stop_color, pos, 0)

    pos = (smargin + bmargin, tmargin, bside, bside)
    pygame.draw.rect(s, color, pos, 2)

    #Hand break box
    if handBreak_state:
	pos = (smargin + 2 * bmargin, tmargin, bside, bside)
	pygame.draw.rect(s, stop_color, pos, 0)

    pos = (smargin + 2 * bmargin, tmargin, bside, bside)
    pygame.draw.rect(s, color, pos, 2)

    #Beacon box
    if beacon_state:
	pos = (smargin + 3 * bmargin, tmargin, bside, bside)
	pygame.draw.rect(s, beacon_color, pos, 0)

    pos = (smargin + 3 * bmargin, tmargin, bside, bside)
    pygame.draw.rect(s, color, pos, 2)

def text_objects(text, font):
    textSurface = font.render(text, True, (255,255,255))
    return textSurface, textSurface.get_rect()

def buttonText():

    mboxy = 175
    mboxx = 325
    dist = 170
    
    font_text = pygame.font.Font('freesansbold.ttf',25)

    #Collision text
    textet, text_box = text_objects('Collision', font_text)
    text_box.center = (mboxx, mboxy)
    s.blit(textet, text_box)

    #Emergency break text
    textet, text_box = text_objects('Emergency', font_text)
    text_box.center = (mboxx + dist, mboxy)
    s.blit(textet, text_box)

    #Hand break text
    textet, text_box = text_objects('Hand Break', font_text)
    text_box.center = (mboxx + 2 * dist, mboxy)
    s.blit(textet, text_box)

    #Beacon text
    textet, text_box = text_objects('Beacon', font_text)
    text_box.center = (mboxx + 3 * dist, mboxy)
    s.blit(textet, text_box)

def main():
    global emergency_state
    global beacon_state
    global handBreak_state
    run = True
    joyInputs()
    collDetect()
    p_relay = rospy.Publisher('relay_controll', Int8MultiArray, queue_size=10)
    p_data = Int8MultiArray()
    p_joy = rospy.Publisher('joy', Joy, queue_size=10)
    joy_data = Joy()

    while run:
	
        s.fill(0)

        for e in pygame.event.get():
            if (e.type == pygame.QUIT or
                    e.type == pygame.KEYDOWN and
                    e.key == pygame.K_ESCAPE):
                run = False
	    if e.type == pygame.MOUSEBUTTONDOWN :
		x, y = e.pos
		if x >= 420 and x <= 570 and y >= 100 and y<= 250:
		    emergency_state = not emergency_state

		    p_data.data.append(emergency_state)
		    p_data.data.append(emergency_state)
		    p_data.data.append(beacon_state)
    		    p_data.data.append(0)

		    p_relay.publish(p_data)

		    del p_data.data[:]
		elif x >= 760 and x <= 910 and y >= 100 and y<= 250:
		    beacon_state = not beacon_state

		    p_data.data.append(emergency_state)
		    p_data.data.append(emergency_state)
		    p_data.data.append(beacon_state)
    		    p_data.data.append(0)

		    p_relay.publish(p_data)

		    del p_data.data[:]

		elif x >= 590 and x <= 740 and y >= 100 and y<= 250:
		    
		    del joy_data.buttons[:]
		    
		    while len(joy_data.axes) < 8:
			joy_data.axes.append(0)

		    while len(joy_data.buttons) < 11:
			joy_data.buttons.append(0)

		    joy_data.buttons[7] = 1

		    p_joy.publish(joy_data)

		    del joy_data.buttons[:]
    	
        drawMeters(s, speed_reference, steering_reference)
	drawBoxes(s, coll_detect, emergency_state)
	buttonText()
        pygame.display.flip()
    
	
        


if __name__ == '__main__':
    rospy.init_node('control_panel', disable_signals=False)
    main()

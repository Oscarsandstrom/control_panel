#!/usr/bin/python

import struct
import rospy
import pygame
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int8MultiArray, Float32MultiArray

# Window variables
WIDTH = 1600
HEIGHT = 1000
FRAMERATE = 60


# Speed variables
speed_reference = 0
speed1 = 0
speed2 = 0
steering_reference = 0
steering_data = 0

permo_speed = 0
permo_steering = 0

# Control variables
coll_detect = False
coll_override = 0

emergency_state = False
emergency_prev = False

handBreak_state = True
handBreak_prev = False

beacon_state = False
beacon_prev = False


# Pygame initialization s as screen with window dimensions and control panel as window name
pygame.init()
s = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Control Panel")
clock = pygame.time.Clock()
clock.tick(FRAMERATE)



def joyCallback(data):
    #Axes variables for later correctional use
    global steering_data
    global speed1
    global speed2

    steering_data = data.axes[0]
    speed1 = data.axes[2]
    speed2 = data.axes[5]

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


#Makes the buttons to toggles for better function
def toggle(val, state, prev):
    if (val == 1) and (not prev):
	state = not state
    prev = (val == 1)
    return state, prev


#Joy subscriber
def joyInputs():
 
    rospy.Subscriber('joy', Joy, joyCallback)



def stopCallback(data):
    global coll_detect
    
    coll_detect = data.data

#Collision detect subscriber
def collDetect():
    
    rospy.Subscriber('lidar_stop', Bool, stopCallback)


#Subscriber for speed and steering reference
def references():

    rospy.Subscriber('referens', Twist, refCallback)

def refCallback(data):
    global steering_reference
    global speed_reference

    speed_reference = data.linear.x
    steering_reference = data.angular.z

#Actual speed and angle speed subscriber
def wvOutputs():
    
    rospy.Subscriber('vw_estimate', Float32MultiArray, speedCallback)


def speedCallback(data):
    global permo_speed
    global permo_steering

    permo_speed = data.data[0]
    permo_steering = data.data[1]

#Draws the steering, speed boxes and shows the referenced speed value and current speed value
def drawMeters(s, speed_reference, steering_reference, permo_speed, permo_steering):
    color = (255, 255, 255)

    speed_color = (0, 0, 255)    
    steering_color = (0, 0, 255)
    wv_color = (255, 0, 0)

    rh = 500
    rw = 150
    rgap = 200

    rx = WIDTH - (2 * rw + 2 * (rgap - rw))
    ry = (HEIGHT - rh)/2


    if permo_speed != 0:
        pos = (rx + 1 * rgap + 1, ry - rw + rh/2, rw - 2, (rh/2) * -permo_speed)
        pygame.draw.rect(s, wv_color, pos, 0)

    if permo_steering != 0:
        pos2 = (rx - 1 * rh/3 + 16 + rh/2,  ry + rh, (rh/8) * -permo_steering, rw - 2)
        pygame.draw.rect(s, wv_color, pos2, 0)

    if steering_reference != 0:
	pos2 = ((rx - 1 * rh/3 + 16 + rh/2) + 250 * -steering_reference, ry + rh, 10, rw - 2)
	pygame.draw.rect(s, steering_color, pos2, 0)
    
    if speed_reference != 0:
        pos = (rx + 1 * rgap + 1, (ry - rw + rh/2) + 250 * -speed_reference, rw - 2, 10)
        pygame.draw.rect(s, speed_color, pos, 0)


    pos = (rx + 1 * rgap, ry - rw, rw, rh + 12)
    pos2 = (rx - 1 * rh/3 + 16, ry + rh, rh + 12, rw)
    pygame.draw.rect(s, color, pos, 2)
    pygame.draw.rect(s, color, pos2, 2)
  
    

#Draws boxes for the buttons and fill them in if they are clicked on
def drawBoxes(s, collision, emergency):
    color = (255, 255, 255)
    reset_color = (0 ,0 ,0)

    stop_color = (255, 0, 0)
    override_color = (11, 102, 35)
    beacon_color = (255, 165, 0)

    #pixel coordinates is the window used later when determening the position
    tmargin = 100
    smargin = 250
    bmargin = 170

    bside = 150


    #Collision box
    if coll_detect:
	#Position of the color filling the box if a collision is detected or in later cases 
	#if the corresponding button is pressed
	pos = (smargin, tmargin, bside, bside)
        pygame.draw.rect(s, stop_color, pos, 0)

    if  coll_override == 1:
	pos = (smargin, tmargin, bside, bside)
        pygame.draw.rect(s, override_color, pos, 0)
    #Position of the frame of the button boxes
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

#Part of the render function of the text
def text_objects(text, font):
    textSurface = font.render(text, True, (255,255,255))
    return textSurface, textSurface.get_rect()

#Renders the text of the buttons
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

    #Initialize subscribers and publishers
    joyInputs()
    collDetect()
    wvOutputs()
    references()
    p_relay = rospy.Publisher('relay_controll', Int8MultiArray, queue_size=10)
    p_data = Int8MultiArray()
    p_joy = rospy.Publisher('joy', Joy, queue_size=10)
    joy_data = Joy()

    #Main loop for the node
    while run:
	

	#Fills the window with the black background
        s.fill(0)


	#A For loop to look for pygame events in the window
        for e in pygame.event.get():
	    #Shutdown of the node
            if (e.type == pygame.QUIT or
                    e.type == pygame.KEYDOWN and
                    e.key == pygame.K_ESCAPE):
                run = False
	    #Makes the buttons clickable and when clicked publishes relevant data
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
		    
		    while len(joy_data.axes) < 8:
			joy_data.axes.append(0)

		    while len(joy_data.buttons) < 11:
			joy_data.buttons.append(0)

		    joy_data.axes[0] = steering_data
		    
		    joy_data.axes[2] = speed1  
    		    joy_data.axes[5] = speed2

		    joy_data.buttons[7] = 1

		    p_joy.publish(joy_data)

		    del joy_data.buttons[:]
    	

	#Updates the window and all the boxes and text
        drawMeters(s, speed_reference, steering_reference, permo_speed, permo_steering)
	drawBoxes(s, coll_detect, emergency_state)
	buttonText()
        pygame.display.flip()
    
	
        


if __name__ == '__main__':
    #Initializing the node for ROS
    rospy.init_node('control_panel', disable_signals=False)
    main()

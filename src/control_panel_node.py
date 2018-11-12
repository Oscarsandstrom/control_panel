#!/usr/bin/python

import struct
import rospy
import pygame
import math
from sensor_msgs.msg import Joy

# WINDOW
WIDTH = 800
HEIGHT = 600
FRAMERATE = 60


# SPEED VARS
speed_reference = 0
steering_reference = 0






def callback(data):
    global steering_reference
    global speed_reference
    
    steering_reference = data.axes[0]
    speed_reference = (-data.axes[5] + data.axes[2])
    

def joyInputs():
 

    rospy.Subscriber('joy', Joy, callback)


def drawMeters(s, speed_reference, steering_reference):
    color = (255, 255, 255)

    speed_color = (0, 255, 0)    
    steering_color = (0, 0, 255)

    rh = 500
    rw = 150
    rgap = 200

    rx = WIDTH - (2 * rw + 2 * (rgap - rw))
    ry = (HEIGHT - rh)/2

    pos = (rx + 1 * rgap, ry, rw, rh)
    pos2 = (ry, rx + 0 * rgap - 1, rh, rw)
    pygame.draw.rect(s, color, pos, 2)
    pygame.draw.rect(s, color, pos2, 2)

    if steering_reference != 0:
	pos2 = (ry + rh/2 + 1, rx + 0 * rgap + 1, (rh/2) * -steering_reference - 1, rw - 2)
	pygame.draw.rect(s, steering_color, pos2, 0)
    
    if speed_reference != 0:
        pos = (rx + 1 * rgap + 1, ry + rh/2 + 1, rw - 2, (rh/4) * -speed_reference - 1)
        pygame.draw.rect(s, speed_color, pos, 0)
  
    




def main():
    run = True
    joyInputs()
    
    # PYGAME
    pygame.init()
    s = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Control Panel")
    clock = pygame.time.Clock()
    clock.tick(FRAMERATE)
    
    while run:
        
	
        s.fill(0)

        for e in pygame.event.get():
            if (e.type == pygame.QUIT or
                    e.type == pygame.KEYDOWN and
                    e.key == pygame.K_ESCAPE):
                run = False 
    	
        drawMeters(s, speed_reference, steering_reference)
        pygame.display.flip()
    
	
        


if __name__ == '__main__':
    rospy.init_node('control_panel', disable_signals=False)
    main()

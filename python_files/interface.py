## Vehicle animation, movement based on SparkFun LSM9DS1 turn connect with Arduino

#!/usr/bin/env python
from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
from time import sleep
import math
import serial

ser = serial.Serial('/dev/cu.usbmodem14201', 4800, timeout = None)
ser.flush()
ser.flushInput()
ser.flushOutput()

ax = ay = az = 0.0
yaw_mode = False

def resize(width, height):
    if height==0:
        height=1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
    sleep(0.05)

def drawText(position, textString):
    font = pygame.font.SysFont ("Courier", 18, True)
    textSurface = font.render(textString, True, (255,255,255,255), (0,0,0,255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

def draw():
    global rquad
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity()
    glTranslatef(0,0.0,-7.0)

    osd_text = "pitch: " + str("{0:.2f}".format(ay)) + ", roll: " + str("{0:.2f}".format(ax), )

    osd_line = osd_text + ", yaw: " + str("{0:.2f}".format(az))

    drawText((-2,-2, 2), osd_line)

    if yaw_mode:                             
        glRotatef(az, 0.0, 1.0, 0.0)  
    else:
        glRotatef(0.0, 0.0, 1.0, 0.0)
    glRotatef(ay ,1.0,0.0,0.0)        # Pitch, rotate around x-axis
    glRotatef(-1*ax ,0.0,0.0,1.0)     # Roll,  rotate around z-axis
    glRotatef(-az ,0.0, 1.0, 0.0)        # Pitch, rotate around x-axis

    glBegin(GL_QUADS)
    glColor3f(0,0,1);
    glVertex3f( 0.2, 0.4,0.6);
    glVertex3f(0.6, 0.5,0.6);
    glVertex3f(0.6, 0.5,0.2);
    glVertex3f( 0.2,0.4,0.2);

    glVertex3f( 0.2,0.4,0.6);
    glVertex3f(0.6,0.2,0.6);
    glVertex3f(0.6,0.2,0.2);
    glVertex3f( 0.2,0.2,0.2);

    glVertex3f( 0.2,0.2,0.6);
    glVertex3f(0.2, 0.4,0.6);
    glVertex3f(0.2,0.4,0.2);
    glVertex3f( 0.2,0.2,0.2);

    glVertex3f(0.6,0.2,0.6);
    glVertex3f(0.6,0.5,0.6);
    glVertex3f(0.6,0.5,0.2);
    glVertex3f( 0.6,0.2,0.2);

    glVertex3f(0.2,0.2,0.6);
    glVertex3f(0.6,0.2,0.6);
    glVertex3f(0.6,0.5,0.6);
    glVertex3f(0.2,0.4,0.6);

    glVertex3f(0.2,0.2,0.2);
    glVertex3f( 0.6,0.2,0.2);
    glVertex3f( 0.6,0.5,0.2);
    glVertex3f( 0.2,0.4,0.2);

    glVertex3f(0.7,0.65,0.6);
    glVertex3f(0.7,0.65,0.2);
    glVertex3f(1.7,0.65,0.2);
    glVertex3f(1.7,0.65,0.6);

    glVertex3f(0.7,0.65,0.6);
    glVertex3f(0.7,0.65,0.2);
    glVertex3f(1.7,0.65,0.2);
    glVertex3f(1.7,0.65,0.6);

    glVertex3f( 2.1,0.2,0.6);
    glVertex3f(2.1,0.2,0.2);
    glVertex3f(1.8,0.2,0.6);
    glVertex3f( 1.8,0.2,0.6);

    glVertex3f(2.1,0.4,0.6);
    glVertex3f(2.1,0.4,0.2);
    glVertex3f(2.1,0.2,0.2);
    glVertex3f(2.1,0.2,0.6);

    glVertex3f(1.8,0.2,0.2);
    glVertex3f(1.8,0.5,0.2);
    glVertex3f(2.1,0.4,0.2);
    glVertex3f(2.1,0.2,0.2);

    glVertex3f(1.8,0.2,0.6);
    glVertex3f(1.8,0.5,0.6);
    glVertex3f(2.1,0.4,0.6);
    glVertex3f(2.1,0.2,0.6);

    glVertex3f( 0.6, 0.5,0.6);
    glVertex3f(0.6, 0.2,0.6);
    glVertex3f(1.8, 0.2, 0.6);
    glVertex3f(1.8,0.5,0.6);

    glVertex3f( 0.6,0.2,0.6);
    glVertex3f(0.6,0.2,0.2);
    glVertex3f(1.8,0.2,0.2);
    glVertex3f( 1.8,0.2,0.6);

    glVertex3f(0.6,0.5,0.2);
    glVertex3f(0.6,0.2,0.2);
    glVertex3f(1.8,0.2,0.2);
    glVertex3f(1.8,0.5,0.2);

    glColor3f(0.3,0.3,0.3);
    glVertex3f( 0.77, 0.63,0.2);
    glVertex3f(0.75, 0.5,0.2);
    glVertex3f(1.2, 0.5, 0.2);
    glVertex3f( 1.22,0.63,0.2);

    glVertex3f(1.27,0.63,.2);
    glVertex3f(1.25,0.5,0.2);
    glVertex3f(1.65,0.5,0.2);
    glVertex3f(1.67,0.63,0.2);

    glColor3f(0,0,1);
    glVertex3f(0.7,0.65,0.2);
    glVertex3f(0.7,0.5,.2);
    glVertex3f(0.75,0.5,0.2);
    glVertex3f(0.77,0.65,0.2);

    glVertex3f(1.2,0.65,0.2);
    glVertex3f(1.2,0.5,.2);
    glVertex3f(1.25,0.5,0.2);
    glVertex3f(1.27,0.65,0.2);

    glVertex3f(1.65,0.65,0.2);
    glVertex3f(1.65,0.5,.2);
    glVertex3f(1.7,0.5,0.2);
    glVertex3f(1.7,0.65,0.2);

    glVertex3f( 0.75, 0.65,0.2);
    glVertex3f(0.75, 0.63,0.2);
    glVertex3f(1.7, 0.63, 0.2);
    glVertex3f( 1.7,0.65,0.2);

    glVertex3f( 0.75, 0.65,0.6);
    glVertex3f(0.75, 0.63,0.6);
    glVertex3f(1.7, 0.63, 0.6);
    glVertex3f( 1.7,0.65,0.6);

    glColor3f(0.3,0.3,0.3);
    glVertex3f( 0.77, 0.63,0.6);
    glVertex3f(0.75, 0.5,0.6);
    glVertex3f(1.2, 0.5, 0.6);
    glVertex3f( 1.22,0.63,0.6);

    glVertex3f(1.27,0.63,.6);
    glVertex3f(1.25,0.5,0.6);
    glVertex3f(1.65,0.5,0.6);
    glVertex3f(1.67,0.63,0.6);

    glColor3f(0,0,1);
    glVertex3f(0.7,0.65,0.6);
    glVertex3f(0.7,0.5,.6);
    glVertex3f(0.75,0.5,0.6);
    glVertex3f(0.77,0.65,0.6);

    glVertex3f(1.2,0.65,0.6);
    glVertex3f(1.2,0.5,.6);
    glVertex3f(1.25,0.5,0.6);
    glVertex3f(1.27,0.65,0.6);

    glVertex3f(1.65,0.65,0.6);
    glVertex3f(1.65,0.5,.6);
    glVertex3f(1.7,0.5,0.6);
    glVertex3f(1.7,0.65,0.6);
    glEnd();


    glBegin(GL_QUADS);

    glColor3f(0.3,0.3,0.3);
    glVertex3f( 0.6, 0.5,0.6);
    glVertex3f(0.6, 0.5,0.2);
    glVertex3f(0.7, 0.65, 0.2);
    glVertex3f( 0.7,0.65,0.6);

    glVertex3f(1.7,0.65,.6);
    glVertex3f(1.7,0.65,0.2);
    glVertex3f(1.8,0.5,0.2);
    glVertex3f(1.8,0.5,0.6);

    glEnd()

    glBegin(GL_TRIANGLES);


    glColor3f(0.3,0.3,0.3);
    glVertex3f( 0.6, 0.5,0.6);
    glVertex3f( 0.7,0.65,0.6);
    glVertex3f(0.7,0.5,0.6);

    glVertex3f( 0.6, 0.5,0.2);
    glVertex3f( 0.7,0.65,0.2);
    glVertex3f(0.7,0.5,0.2);

    glVertex3f( 1.7, 0.65,0.2);
    glVertex3f( 1.8,0.5,0.2);
    glVertex3f( 1.7,0.5,0.2);

    glVertex3f( 1.7, 0.65,0.6);
    glVertex3f( 1.8,0.5,0.6);
    glVertex3f(1.7,0.5,0.6);

    glEnd();

    glColor3f(0.7,0.7,0.7);
    glPushMatrix();
    glBegin(GL_LINE_STRIP);
    t = 0
    while(t<360):
        glVertex3f(0.6,0.2,0.62);
        glVertex3f(0.6+(0.08*(math.cos(((t)*3.14)/180))),0.2+(0.08*(math.sin(((t)*3.14)/180))),0.62);
        t = t + 40
    glEnd();

    glBegin(GL_LINE_STRIP);
    t = 0
    while(t<360):
        glVertex3f(0.6,0.2,0.18);
        glVertex3f(0.6+(0.08*(math.cos(((t)*3.14)/180))),0.2+(0.08*(math.sin(((t)*3.14)/180))),0.18);
        t = t + 40
    glEnd();

    glBegin(GL_LINE_STRIP);
    t = 0
    while(t<360):
        glVertex3f(1.7,0.2,0.18);
        glVertex3f(1.7+(0.08*(math.cos(((t)*3.14)/180))),0.2+(0.08*(math.sin(((t)*3.14)/180))),0.18);
        t = t + 40
    glEnd();

    glBegin(GL_LINE_STRIP);
    t = 0
    while(t<360):
        glVertex3f(1.7,0.2,0.62);
        glVertex3f(1.7+(0.08*(math.cos(((t)*3.14)/180))),0.2+(0.08*(math.sin(((t)*3.14)/180))),0.62);
        t = t + 40
    glEnd();

    glTranslatef(0.6,0.2,0.6);
    glColor3f(0,0,0);

    glTranslatef(0,0,-0.4);

    glTranslatef(1.1,0,0);

    glTranslatef(0,0,0.4);
    glPopMatrix();

def read_data():
    global ax, ay, az
    ax = ay = az = 0.0
    line_done = 0
    ser.write(b".") 
    line = ser.readline()
    angles = line.split(b",")
    if len(angles) == 3:
        ax = float(angles[0])
        ay = float(angles[1])
        az = float(angles[2])
        line_done = 1

def main():
    global yaw_mode

    animation_flag = OPENGL|DOUBLEBUF

    pygame.init()
    screen = pygame.display.set_mode((640,480), animation_flag)
    resize(640,480)
    init()
    frames = 0
    ticks = pygame.time.get_ticks()
    print(ax)
    while 1:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            pygame.quit()  
            break
        
        ser.write(b"z")
        read_data()
        draw()

        pygame.display.flip()
        frames = frames+1

    print ("fps:  %d" % ((frames*1000)/(pygame.time.get_ticks()-ticks)))
    ser.close()

if __name__ == '__main__': main()

#!/usr/bin/env python3

import sys, pygame
import pygame.gfxdraw
import pygame.freetype
from pygame_vkeyboard import *
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Vector3

COLOR_INACTIVE = pygame.Color('lightskyblue3')
COLOR_ACTIVE = pygame.Color('dodgerblue2')


def draw_rounded_rect(surface, rect, color, corner_radius):
    ''' Draw a rectangle with rounded corners.
    Would prefer this: 
        pygame.draw.rect(surface, color, rect, border_radius=corner_radius)
    but this option is not yet supported in my version of pygame so do it ourselves.

    We use anti-aliased circles to make the corners smoother
    '''
    if rect.width < 2 * corner_radius or rect.height < 2 * corner_radius:
        raise ValueError(f"Both height (rect.height) and width (rect.width) must be > 2 * corner radius ({corner_radius})")

    # need to use anti aliasing circle drawing routines to smooth the corners
    pygame.gfxdraw.aacircle(surface, rect.left+corner_radius, rect.top+corner_radius, corner_radius, color)
    pygame.gfxdraw.aacircle(surface, rect.right-corner_radius-1, rect.top+corner_radius, corner_radius, color)
    pygame.gfxdraw.aacircle(surface, rect.left+corner_radius, rect.bottom-corner_radius-1, corner_radius, color)
    pygame.gfxdraw.aacircle(surface, rect.right-corner_radius-1, rect.bottom-corner_radius-1, corner_radius, color)

    pygame.gfxdraw.filled_circle(surface, rect.left+corner_radius, rect.top+corner_radius, corner_radius, color)
    pygame.gfxdraw.filled_circle(surface, rect.right-corner_radius-1, rect.top+corner_radius, corner_radius, color)
    pygame.gfxdraw.filled_circle(surface, rect.left+corner_radius, rect.bottom-corner_radius-1, corner_radius, color)
    pygame.gfxdraw.filled_circle(surface, rect.right-corner_radius-1, rect.bottom-corner_radius-1, corner_radius, color)

    rect_tmp = pygame.Rect(rect)

    rect_tmp.width -= 2 * corner_radius
    rect_tmp.center = rect.center
    pygame.draw.rect(surface, color, rect_tmp)

    rect_tmp.width = rect.width
    rect_tmp.height -= 2 * corner_radius
    rect_tmp.center = rect.center
    pygame.draw.rect(surface, color, rect_tmp)

class Button:
    def __init__(self, surface, rect, text):
        self.surface = surface
        self.rect = rect
        self.text = text

        circular14 = pygame.freetype.Font("/home/pi/.fonts/CircularStd-Bold.ttf",14)
        p_rect = pygame.rect.Rect(rect)
        draw_rounded_rect(surface, p_rect, (200,200,200),18)

    def render_button(self, mouse):
        circular14 = pygame.freetype.Font("/home/pi/.fonts/CircularStd-Bold.ttf",14)

        p_rect = pygame.rect.Rect(self.rect)
        if mouse[0] < self.rect[0]+self.rect[2] and mouse[0]> self.rect[0] and mouse[1] < self.rect[1]+self.rect[3] and mouse[1] > self.rect[1]:
            draw_rounded_rect(self.surface, p_rect, (255,255,255),18)
        else:
            draw_rounded_rect(self.surface, p_rect, (200,200,200),18)

        circular14.render_to(self.surface, (self.rect[0]+10, self.rect[1]+15), self.text, (0,0,0))
    
    def check_click(self, mouse):
        if mouse[0] < self.rect[0]+self.rect[2] and mouse[0]> self.rect[0] and mouse[1] < self.rect[1]+self.rect[3] and mouse[1] > self.rect[1]:
            return 1
        else:
            return 0
        
class ControlButton:
    def __init__(self, surface, rect, text):
        self.surface = surface
        self.rect = rect
        self.text = text

        circular14 = pygame.freetype.Font("/home/pi/.fonts/CircularStd-Bold.ttf",26)
        p_rect = pygame.rect.Rect(rect)
        draw_rounded_rect(surface, p_rect, (200,200,200),18)

    def render_button(self, mouse):
        circular14 = pygame.freetype.Font("/home/pi/.fonts/CircularStd-Bold.ttf",26)

        p_rect = pygame.rect.Rect(self.rect)
        t_rect = circular14.get_rect(self.text, size = 26)
        t_rect.center = p_rect.center 

        if mouse[0] < self.rect[0]+self.rect[2] and mouse[0]> self.rect[0] and mouse[1] < self.rect[1]+self.rect[3] and mouse[1] > self.rect[1]:
            draw_rounded_rect(self.surface, p_rect, (255,255,255),18)
        else:
            draw_rounded_rect(self.surface, p_rect, (200,200,200),18)

        circular14.render_to(self.surface, t_rect, self.text, (0,0,0))
    
    def check_click(self, mouse):
        if mouse[0] < self.rect[0]+self.rect[2] and mouse[0]> self.rect[0] and mouse[1] < self.rect[1]+self.rect[3] and mouse[1] > self.rect[1]:
            return 1
        else:
            return 0
    
class IconButton:
    def __init__(self, surface, pos, radius, icon):
        self.surface = surface
        self.pos = pos
        self.icon = pygame.transform.scale(pygame.image.load(icon), (radius, radius) )

        self.radius = radius

    def render_button(self, mouse):
        if mouse[0] < self.pos[0]+self.radius and mouse[0]> self.pos[0]-self.radius and mouse[1] < self.pos[1]+self.radius and mouse[1] > self.pos[1] - self.radius:
            pygame.draw.circle(self.surface, (200,200,200), self.pos, self.radius)
            self.surface.blit(self.icon, (self.pos[0]-self.radius/2.4, self.pos[1]-self.radius/2)) 

        else:
            pygame.draw.circle(self.surface, (255,255,255), self.pos, self.radius)
            self.surface.blit(self.icon, (self.pos[0]-self.radius/2.4, self.pos[1]-self.radius/2)) 

    
    def check_click(self, mouse):
        if mouse[0] < self.pos[0]+self.radius and mouse[0]> self.pos[0]-self.radius and mouse[1] < self.pos[1]+self.radius and mouse[1] > self.pos[1] - self.radius:
            return 1
        else:
            return 0


class ServiceMenuItem:
    def __init__(self, surface, id, package, service, interface):
        self.w = 230
        self.h = 60
        self.package = package
        self.service = service
        self.surface = surface
        self.id = id
        self.rect = pygame.rect.Rect(40,100+80*id,self.w,self.h)
        self.button = IconButton(surface, (self.w+10,int(self.h/2)+100+80*id), 20, "/home/pi/control_ws/src/control/play.png")
        self.submit = 0
        layout = VKeyboardLayout(VKeyboardLayout.QWERTY)
        self.keyboard = VKeyboard(self.surface, self.consumer, layout)
        self.window = False
        
    def render(self, mouse, events):
        if mouse[0] < self.rect[0]+self.rect[2] and mouse[0]> self.rect[0] and mouse[1] < self.rect[1]+self.rect[3] and mouse[1] > self.rect[1]:
            draw_rounded_rect(self.surface, self.rect, (190,190,190),15)
        else:
            draw_rounded_rect(self.surface, self.rect, (230,230,230),15)

        self.circular16 = pygame.freetype.Font("/home/pi/.fonts/CircularStd-Bold.ttf",16)
        self.circular12 = pygame.freetype.Font("/home/pi/.fonts/CircularStd-Bold.ttf",12)        
        self.circular12.render_to(self.surface, (60, 116+80*self.id), self.package, (50,50,50))        
        self.circular16.render_to(self.surface, (60, 130+80*self.id), self.service, (0,0,0))
        self.button.render_button(mouse)
        self.keyboard.update(events)
        if self.window:
            self.keyboard.draw(self.surface)


    def check_click(self, mouse):

        if self.button.check_click(mouse):
            return 2
        elif mouse[0] < self.rect[0]+self.rect[2] and mouse[0]> self.rect[0] and mouse[1] < self.rect[1]+self.rect[3] and mouse[1] > self.rect[1]:
            self.popup()
            return 1
        else:
            return 0

    def popup(self):
        self.window=True
        width = 800
        height = 600
        rect = pygame.rect.Rect((1872-width)/2, (1000-height)/2-100, width, height/2)
        draw_rounded_rect(self.surface,rect,(200,200,200), 15)
        
        txt_rect = pygame.rect.Rect((1872-width)/2+20, (1000-height)/2+100, width-40, 35)
        draw_rounded_rect(self.surface, txt_rect, (255,255,255),15)

        self.circular12.render_to(self.surface, ((1872-width)/2+20, (1000-height)/2+20), self.package, (50,50,50))        
        self.circular16.render_to(self.surface, ((1872-width)/2+20, (1000-height)/2+40), self.service, (0,0,0))

        submit_rect = pygame.rect.Rect((1872-800)/2+600, (1000-600)/2+235, 800-620, 45)
        self.submit = Button(self.surface, submit_rect,"Submit")
        # self.submit.render_button(mouse)
            
    def consumer(self, text):
        width = 800
        height = 600
        txt_rect = pygame.rect.Rect((1872-width)/2+20, (1000-height)/2+100, width-40, 35)
        draw_rounded_rect(self.surface, txt_rect, (255,255,255),15)
        self.circular12.render_to(self.surface, ((1872-width)/2+30, (1000-height)/2+110), text, (50,50,50))    
    
class Interface:
    def __init__(self, pygame):
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odomCallback)
        self.js_sub = rospy.Subscriber("joint_states", JointState, self.jsCallback)
        self.enc = [0,0]
        self.odom = Pose()
        self.odom.position.x = 0
        self.odom.position.y = 0
        self.odom.orientation.z = 0
        self.pygame = pygame
        self.pygame.font.init()
        self.modules = rospy.get_param("/interface/modules")
        self.buttons = rospy.get_param("/interface/buttons")

        print("MOD:"+str(self.modules['odom']))
        print(self.pygame.font.match_font("CircularStd-Bold"))
        self.circular20 = self.pygame.freetype.Font("/home/pi/.fonts/CircularStd-Bold.ttf",26)
        self.circular14= self.pygame.freetype.Font("/home/pi/.fonts/CircularStd-Bold.ttf",20)

        self.size = self.width, self.height = 1872, 1300
        speed = [2, 2]
        self.BLACK = (0, 0, 0)
        self.WHITE = (255,255,255)

        self.screen = pygame.display.set_mode(self.size)

        self.menu = []
        self.control_buttons = []

        for i in range(len(self.buttons['id'])):
            b_rect = self.pygame.rect.Rect(330,100+120*i,360,100)
            b = ControlButton(self.screen, b_rect, self.buttons['text'][i])
            self.control_buttons.append(b)
            
        self.reset()
        
    def odomCallback(self, msg):
        self.odom = msg.pose.pose
        
    def jsCallback(self, msg):
        self.enc = msg.position

    def reset(self):

        srv_rect = self.pygame.rect.Rect(20,20,270,self.height-40)
        console_rect = self.pygame.rect.Rect(730,20,self.width-730-20,self.height-40)
        cntrl_rect = self.pygame.rect.Rect(310,20,400,self.height-40)

        self.screen.fill(self.WHITE)
        draw_rounded_rect(self.screen, srv_rect, self.BLACK, 20)
        draw_rounded_rect(self.screen, cntrl_rect, self.BLACK, 20)
        draw_rounded_rect(self.screen, console_rect, self.BLACK, 20)

        self.circular20.render_to(self.screen, (50,50), "ROS Services", self.WHITE)
        self.circular20.render_to(self.screen, (760,50), "Console", self.WHITE)
        self.circular20.render_to(self.screen, (340,50), "Basic Controls", self.WHITE)

        self.menu.append(ServiceMenuItem(self.screen, 0, "turtlesim", "translate", self))
        self.menu.append(ServiceMenuItem(self.screen, 1, "controller", "forward", self))
        self.menu.append(ServiceMenuItem(self.screen, 2, "controller", "backward", self))
        
    def info_module(self, name, info_array, x, y):
        info_rect = self.pygame.rect.Rect(x,y,200,200)
        info_title_rect = self.pygame.rect.Rect(x+10,y+10,40+15*len(name),40)
        
        draw_rounded_rect(self.screen, info_rect, self.WHITE, 20)
        draw_rounded_rect(self.screen, info_title_rect, self.BLACK, 15)

        self.circular20.render_to(self.screen, (x+25,y+20), name, self.WHITE)
        for i in range(int(len(info_array)/2)):
            self.circular14.render_to(self.screen, (x+20,y+60+20*i), info_array[i*2]+str(info_array[2*i+1]), self.BLACK)
          

    def refresh(self):
        mouse = self.pygame.mouse.get_pos() 
        events = self.pygame.event.get()
        for m in self.menu:
            m.render(mouse, events)
        for b in self.control_buttons:
            b.render_button(mouse)
        for event in events:
            if event.type==self.pygame.QUIT:
                self.pygame.quit()
                sys.exit()
            if event.type == self.pygame.MOUSEBUTTONDOWN: 
                for m in self.menu:
                    m.check_click(mouse)
                for i in range(len(self.control_buttons)):
                    if self.control_buttons[i].check_click(mouse):
                        pubtype = 0
                        if self.buttons['type'][i] == "Twist":
                            pubtype = Twist
                        if self.buttons['type'][i] == "Twist":
                            print(self.buttons['publishes'][i]['linear']['x'])
                            publishes = Twist(linear = Vector3(x = self.buttons['publishes'][i]['linear']['x'], y = self.buttons['publishes'][i]['linear']['y'], z = self.buttons['publishes'][i]['linear']['z']), angular = Vector3(x = self.buttons['publishes'][i]['angular']['x'], y = self.buttons['publishes'][i]['angular']['y'], z = self.buttons['publishes'][i]['angular']['z']))

                        pub = rospy.Publisher(self.buttons['publisher'][i], pubtype, queue_size=0)
                        pub.publish(publishes)
                        
                    
        console_rect = self.pygame.rect.Rect(730,20,self.width-730-20,self.height-40)
        draw_rounded_rect(self.screen, console_rect, self.BLACK, 20)
        self.circular20.render_to(self.screen, (760,50), "Console", self.WHITE)

        i = 0
        j = 0
        if self.modules['name']:
            self.info_module("Robot",["name: ", "turtlebot", "ip: ", "192.0.0.1","type: ","diff_drive"],760+220*i,100+220*j) 
            i+=1
            if i>3:
                i = 0
                j+=1
        if self.modules['encoders']:
            self.info_module("Encoders",["Left: ", round(self.enc[0],4), "Right: ", round(self.enc[1],4)],760+220*i,100+220*j)
            i+=1
            if i>3:
                i = 0
                j+=1
        if self.modules['odom']:
            self.info_module("Odom",["x: ", round(self.odom.position.x,4), "y: ", round(self.odom.position.y,4),"theta: ",round(self.odom.orientation.z,4)],760+220*i,100+220*j)
            i+=1
            if i>3:
                i = 0
                j+=1
                
        
            
        







        


        self.pygame.display.update()



if __name__ == '__main__':
    pygame.init()
    rospy.init_node("interface")
    interface = Interface(pygame)
    

    while True:
            interface.refresh()
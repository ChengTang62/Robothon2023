import functions 
import take_pic 
import get_coord
import get_orien

class TaskSequence(object):
    def __init__(self):
        #rospy.init_node("take_pic")

        self.rc = functions.RobotControl()
        self.tp = take_pic.TakePicture()
        self.gc = get_coord.GetCoord()
        self.go = get_orien.GetOrien()
        self.v = 1
        

    def reset(self):
        self.rc.reset()

    def task1(self):
        self.rc.cart_to_blue_button()
        self.rc.push_button()

    def task2(self):
        self.rc.cart_to_screen()
        self.rc.cart_to_slider()
        self.rc.adjust_slider_mid()
        self.rc.cart_to_screen()
        dist = self.tp.get_slider_dist()
        if dist > 0:
            self.rc.adjust_slider_same_side(dist)
        elif dist < 0:
            self.rc.adjust_slider_other_side(dist)
        
        self.reset()

    def task3(self):
        self.rc.cart_to_plug()
        self.rc.unplug()
        self.rc.cart_to_plugin()
        self.rc.plugin()

    def task4_1(self):
        self.rc.cart_to_door()
        self.rc.open_door()
    
    def task4_2(self):
        self.rc.cart_to_probe_grab()
        self.rc.pull_out_probe()
        self.rc.cart_to_probe_in()
        self.rc.move_cart(0,0,0.028,0.02)
        self.rc.move_cart(0,0,-0.2,0.5)
    
    def task5(self):
        self.rc.reset()
        self.rc.cart_to_probe_stage1()
        self.rc.move_cart(-0.12,0,0,0.1)
        self.rc.ungrip()

    def task6(self):
        self.rc.reset()
        self.rc.cart_to_red_button()
        self.rc.push_button()
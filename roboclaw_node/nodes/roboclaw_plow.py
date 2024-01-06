#!/usr/bin/env python
from math import pi, cos, sin

import diagnostic_msgs
import diagnostic_updater
import roboclaw_driver.roboclaw_driver as roboclaw
import rospy
from geometry_msgs.msg import Twist



__author__ = "bwbazemore@uga.edu (Brad Bazemore)"




class Movement:
    def __init__(self, address, max_duty):
        self.twist = None
        self.address = address
        self.MAX_DUTY = max_duty
        self.last_set_move_time = rospy.get_rostime()
        self.stopped = True
        self.last_yn0 = None
        self.last_y0 = None
        self.curent_yn0 = None
        self.curent_y0 = None

    def run(self):
        if self.twist is None:
            return
        self.last_set_move_time = rospy.get_rostime()
        if self.twist.linear.x != 0 :
            self.stopped = False
            self.lastyn0 = self.curent_yn0
            self.curent_yn0 = rospy.get_rostime()

        linear_x = self.twist.linear.x
        #if linear_y > self.MAX_DUTY:
        #    linear_y = self.MAX_DUTY
        #if linear_y < -self.MAX_DUTY:
        #    linear_y = -self.MAX_DUTY

        try:
            if linear_x >= 0:
                roboclaw.BackwardM1(self.address, int(linear_x * self.MAX_DUTY))
            elif linear_x < 0:
                roboclaw.ForwardM1(self.address, self.MAX_DUTY)
            # if linear_x is 0 :
            #     self.lasty0 = self.curent_y0
            #     self.curent_y0 = rospy.get_rostime()
            #     roboclaw.DutyM1M2(self.address, 0, 0)
            #     self.linear_y = 0

            # else:
            #     #if (self.curent_yn0 - self.lastyn0) <= une valeur en ms and (self.curent_y0 - self.lasty0) <= meme valeur:
            #     #    self.linear_y = int(linear_y * self.MAX_DUTY * 3 if (3 * self.MAX_DUTY * linear_y) <= 127 else 127 ) #verfifier le vrai max duty
            #     #else:
            #     self.linear_y = int(linear_y * self.MAX_DUTY)
            #     roboclaw.DutyM1M2(self.address, self.linear_y, self.linear_y) #changer pas de vitesse juste duty
        except OSError as e:
            rospy.logwarn("DutyM1M2 OSError: %d", e.errno)
            rospy.logdebug(e)

class Node:
    def __init__(self):

        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
                       0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
                       0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
                       0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
                       0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
                       0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}

        rospy.init_node("roboclaw_plow") #ne change pas
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to roboclaw")
        dev_name = rospy.get_param("~dev", "/dev/rcPlow")
        baud_rate = int(rospy.get_param("~baud", "115200"))

        self.address = int(rospy.get_param("~address", "130"))
        if self.address > 0x87 or self.address < 0x80:
            rospy.logfatal("Address out of range")
            rospy.signal_shutdown("Address out of range")

        # TODO need someway to check if address is correct
        try:
            roboclaw.Open(dev_name, baud_rate)
        except Exception as e:
            rospy.logfatal("Could not connect to Roboclaw")
            rospy.logdebug(e)
            rospy.signal_shutdown("Could not connect to Roboclaw")

        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("Roboclaw")
        self.updater.add(diagnostic_updater.
                         FunctionDiagnosticTask("Vitals", self.check_vitals))

        try:
            version = roboclaw.ReadVersion(self.address)
        except Exception as e:
            rospy.logwarn("Problem getting roboclaw version")
            rospy.logdebug(e)

        if not version[0]:
            rospy.logwarn("Could not get version from roboclaw")
        else:
            rospy.logdebug(repr(version[1]))


        roboclaw.ForwardM1(self.address, 0)
        self.MAX_DUTY = rospy.get_param("~max_duty", "30")  #check avec la manette
        self.STOP_MOVEMENT = rospy.get_param("~stop_movement", "true")
        self.movement = Movement(self.address, self.MAX_DUTY)
        self.last_set_move_time = rospy.get_rostime()
        
        rospy.Subscriber("cmd_vel_plow", Twist, self.cmd_vel_callback, queue_size=1)

        rospy.sleep(1)

        rospy.logdebug("dev %s", dev_name)
        rospy.logdebug("baud %d", baud_rate)
        rospy.logdebug("address %d", self.address)
        rospy.logdebug("max_duty %f", self.MAX_DUTY)


    def run(self):
        rospy.loginfo("Starting plow linear motor")
        r_time = rospy.Rate(30)
        while not rospy.is_shutdown():

            # stop movement if robot doesn't receive commands for 1 sec
            if (self.STOP_MOVEMENT and not self.movement.stopped and rospy.get_rostime().to_sec() - self.movement.last_set_move_time.to_sec() > 1): #set speed time enlever?
                rospy.loginfo("Did not get command for 1 second, stopping")
                rospy.logerr("HERE")
                try:
                    roboclaw.ForwardM1(self.address, 0)
                    #roboclaw.ForwardM2(self.address, 0) ## remplacer par la fonction dutym1m2
                except OSError as e:
                    rospy.logerr("Could not stop")
                    rospy.logdebug(e)
                self.movement.stopped = True
            
            self.updater.update()
            self.movement.run()

            r_time.sleep()

    def cmd_vel_callback(self, twist):
        self.movement.last_set_move_time = rospy.get_rostime()
        self.movement.twist = twist

    # TODO: Need to make this work when more than one error is raised
    def check_vitals(self, stat):
        try:
            status = roboclaw.ReadError(self.address)[1]
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
            return
        state, message = self.ERRORS[status]
        stat.summary(state, message)
        try:
            stat.add("Main Batt V:", float(roboclaw.ReadMainBatteryVoltage(self.address)[1] / 10))
            stat.add("Logic Batt V:", float(roboclaw.ReadLogicBatteryVoltage(self.address)[1] / 10))
            stat.add("Temp1 C:", float(roboclaw.ReadTemp(self.address)[1] / 10))
            stat.add("Temp2 C:", float(roboclaw.ReadTemp2(self.address)[1] / 10))
            _, cur1, cur2 = roboclaw.ReadCurrents(self.address)
            stat.add("Current1 A:", float(cur1/100.0))
            stat.add("Current2 A:", float(cur2/100.0))
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
        return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        rospy.loginfo("Shutting down")
        try:
            roboclaw.ForwardM1(self.address, 0)
            roboclaw.ForwardM2(self.address, 0)
        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                roboclaw.ForwardM1(self.address, 0)
                roboclaw.ForwardM2(self.address, 0)
            except OSError as e:
                rospy.logerr("Could not shutdown motors!!!!")
                rospy.logdebug(e)


if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")

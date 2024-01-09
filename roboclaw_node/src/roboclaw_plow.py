#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from driver.roboclaw_driver import RoboclawDriver


class Movement:
    def __init__(self, driver, max_duty):
        self.twist = None
        self.driver = driver
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
        self.stopped = (self.twist.linear.x == 0)

        linear_x = self.twist.linear.x
        rospy.logdebug("Duty:%d ", linear_x)

        try:
            if linear_x > 0:
                self.driver.ForwardM1(self.MAX_DUTY)
            elif linear_x < 0:
                self.driver.BackwardM1(self.MAX_DUTY)
            else:
                self.driver.ForwardM1(0)

        except OSError as e:
            rospy.logwarn("BackwardM1/ForwardM1 OSError: %d", e.errno)
            rospy.logdebug(e)

class Node:

    MAX_ADDRESS = 0x87
    MIN_ADDRESS = 0x80
   
    def __init__(self):
        rospy.init_node(rospy.get_param("~name", "roboclaw_plow"))
        self.dev_name = rospy.get_param("~dev", "/dev/rcPlow")
        self.address = self.get_address("~address", "130")
        self.baud_rate = int(rospy.get_param("~baud", "115200"))
        self.driver = self.setup_device(self.dev_name, self.address, self.baud_rate)

        self.reset_device()
        self.parameters_setup()
        self.publishers_and_subscribers_setup()
        rospy.on_shutdown(self.shutdown)

    def get_address(self, param_name, default_value):
        address = int(rospy.get_param(param_name, default_value))
        if address > self.MAX_ADDRESS or address < self.MIN_ADDRESS:
            rospy.logfatal("Address " + str(address) + " out of range. Must be in range "
                + str(self.MIN_ADDRESS) + " - " + str(self.MAX_ADDRESS)
                )
            rospy.signal_shutdown("Address out of range")
            return None
        return address

    def setup_device(self, dev_name, address, baud_rate):
        try:
            driver = RoboclawDriver(dev_name, baud_rate, address)
        except Exception as e:
            rospy.logfatal("Could not connect to Roboclaw at " + dev_name + " with address " + str(address))
            rospy.logdebug(e)
            rospy.signal_shutdown("Could not connect to Roboclaw")
            return

        version = self.get_version_and_log(driver)
        if version is not None:
            if not version[0]:
                rospy.logwarn("Could not get version from Roboclaw at address: " + str(address))
            else:
                rospy.logdebug(repr(version[1]))

        return driver
    
    def get_version_and_log(self, driver):
        version = None
        try:
            version = driver.ReadVersion()
        except Exception as e:
            rospy.logwarn(
                "Problem getting Roboclaw version from address: " + str(driver.address)
            )
            rospy.logdebug(e)
        return version

    def reset_device(self):
        self.driver.ForwardM1(0)

    def parameters_setup(self):
        self.RATE = rospy.get_param("~rate", 20)
        self.MAX_DUTY = rospy.get_param("~max_duty", 100)
        self.STOP_MOVEMENT = rospy.get_param("~stop_movement", "true")

    def publishers_and_subscribers_setup(self):
        self.movement = Movement(self.driver, self.MAX_DUTY)
        self.last_set_speed_time = rospy.get_rostime()
        rospy.Subscriber("cmd_vel_plow", Twist, self.cmd_vel_callback, queue_size=1)

    def run(self):
        rospy.loginfo("Starting plow linear motor")
        r_time = rospy.Rate(self.RATE)

        while not rospy.is_shutdown():
            # stop movement if robot doesn't receive commands for 1 sec
            if (
                self.STOP_MOVEMENT and not self.movement.stopped
                and rospy.get_rostime().to_sec() - self.movement.last_set_move_time.to_sec() > 1
            ):
                rospy.loginfo("Did not get command for 1 second, stopping")
                try:
                    self.driver.ForwardM1(0)
                    self.movement.stopped = True
                except OSError as e:
                    rospy.logerr("Could not stop")
                    rospy.logdebug(e)
            
            # Execute last command
            self.movement.run()

            r_time.sleep()

    def cmd_vel_callback(self, twist):
        self.movement.last_set_move_time = rospy.get_rostime()
        self.movement.twist = twist

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        rospy.loginfo("Shutting down")
        try:
            self.driver.StopMotors()
        except OSError as e:
            rospy.logerr("Shutdown did not work trying again")
            rospy.logdebug(e)
            try:
                self.driver.StopMotors()
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

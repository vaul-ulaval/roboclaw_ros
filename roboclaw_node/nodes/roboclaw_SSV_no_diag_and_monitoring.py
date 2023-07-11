#!/usr/bin/env python
from math import pi, cos, sin

from roboclaw_driver.roboclaw_driver import RoboclawDriver
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

__author__ = "bwbazemore@uga.edu (Brad Bazemore)"

# TODO need to find some better was of handling OSerror 11 or preventing it, any ideas?

class EncoderOdom:
    TICKS_PER_METER = None
    BASE_WIDTH = None
    ENCODER_JUMP_THRESHOLD = 20000

    def __init__(self, ticks_per_meter, base_width):
        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = base_width
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_front_left = 0
        self.last_enc_front_right = 0
        self.last_enc_rear_left = 0
        self.last_enc_rear_right = 0
        self.last_enc_time = rospy.Time.now()
        self.vel_theta = 0

    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, enc_front_left, enc_front_right, enc_rear_left, enc_rear_right):   
        # Calculate the wheel speeds from the encoder readings
        speed_front_left = (enc_front_left - self.last_enc_front_left) / self.TICKS_PER_METER
        speed_front_right = (enc_front_right - self.last_enc_front_right) / self.TICKS_PER_METER
        speed_rear_left = (enc_rear_left - self.last_enc_rear_left) / self.TICKS_PER_METER
        speed_rear_right = (enc_rear_right - self.last_enc_rear_right) / self.TICKS_PER_METER

        self.last_enc_front_left = enc_front_left
        self.last_enc_front_right = enc_front_right
        self.last_enc_rear_left = enc_rear_left
        self.last_enc_rear_right = enc_rear_right

        # Calculate the average speed of the left and right wheels
        speed_left = (speed_front_left + speed_rear_left) / 2.0
        speed_right = (speed_front_right + speed_rear_right) / 2.0

        # Calculate the change in orientation
        delta_theta = (speed_right - speed_left) / self.BASE_WIDTH

        current_time = rospy.Time.now()
        delta_time = (current_time - self.last_enc_time).to_sec()
        self.last_enc_time = current_time

        self.cur_x += (speed_right + speed_left) / 2.0 * cos(self.cur_theta) * delta_time
        self.cur_y += (speed_right + speed_left) / 2.0 * sin(self.cur_theta) * delta_time
        self.cur_theta = self.normalize_angle(self.cur_theta + delta_theta * delta_time)

        vel_x = 0.0 if abs(delta_time) < 0.000001 else (speed_right + speed_left) / 2.0
        vel_theta = 0.0 if abs(delta_time) < 0.000001 else delta_theta

        self.vel_theta = vel_theta
        return vel_x, vel_theta

    def update_publish(self, enc_front_left, enc_front_right, enc_rear_left, enc_rear_right):
        encoders = {
            "front_left": enc_front_left,
            "front_right": enc_front_right,
            "rear_left": enc_rear_left,
            "rear_right": enc_rear_right
        }

        for encoder_name, reading in encoders.items():
            last_reading = getattr(self, "last_enc_" + encoder_name)

            if abs(reading - last_reading) > self.ENCODER_JUMP_THRESHOLD:
                rospy.logerr("Ignoring " + encoder_name + " encoder jump: cur " + reading + ", last " + last_reading)
                return

            setattr(self, "last_enc_" + encoder_name, reading)

        vel_x, vel_theta = self.update(enc_front_left, enc_front_right, enc_rear_left, enc_rear_right)
        self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
        current_time = rospy.Time.now()

        br = tf.TransformBroadcaster()
        br.sendTransform((cur_x, cur_y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, cur_theta),
                         current_time,
                         "base_link",
                         "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)

class Movement:
    def __init__(self, drivers, addresses, max_speed, base_width, ticks_per_meter, max_accel):
        self.twist = None
        self.drivers = drivers
        self.addresses = addresses
        self.MAX_SPEED = max_speed
        self.BASE_WIDTH = base_width
        self.TICKS_PER_METER = ticks_per_meter
        self.MAX_ACCEL = max_accel
        self.last_set_speed_time = rospy.get_rostime()
        self.vr_ticks = 0
        self.vl_ticks = 0
        self.stopped = True

    def run(self):
        if self.twist is None:
            return
        # self.last_set_speed_time = rospy.get_rostime()

        if self.twist.linear.x != 0 or self.twist.angular.z != 0:
            self.stopped = False

        linear_x = self.twist.linear.x
        if linear_x > self.MAX_SPEED:
            linear_x = self.MAX_SPEED
        if linear_x < -self.MAX_SPEED:
            linear_x = -self.MAX_SPEED

        #vr = linear_x + self.twist.angular.z * self.BASE_WIDTH  # m/s
        #vl = linear_x - self.twist.angular.z * self.BASE_WIDTH
        vr = linear_x - self.twist.angular.z
        vl = linear_x + self.twist.angular.z
        self.twist = None

        vr_ticks = int(vr * self.TICKS_PER_METER)  # ticks/s
        vl_ticks = int(vl * self.TICKS_PER_METER)

        rospy.logdebug("vr_ticks:%d vl_ticks: %d", vr_ticks, vl_ticks)

        if self.drivers is not None:
            try:
                for driver, address in zip(self.drivers, self.addresses):
                    if vr_ticks is 0 and vl_ticks is 0:
                        driver.ForwardM1(address, 0)
                        driver.ForwardM2(address, 0)
                        self.vr_ticks = 0
                        self.vl_ticks = 0
                    else:
                        self.vr_ticks = vr_ticks
                        self.vl_ticks = vl_ticks
                        driver.SpeedM1M2(address, int(self.vr_ticks), int(self.vl_ticks))
            except OSError as e:
                rospy.logwarn("SpeedM1M2 OSError: %d", e.errno)
                rospy.logdebug(e)


class Node:

    MAX_ADDRESS = 0x87
    MIN_ADDRESS = 0x80

    def __init__(self):
        rospy.init_node("roboclaw_motors")
        self.dev_names = [rospy.get_param("~dev_front", "/dev/rcFront"), rospy.get_param("~dev_rear", "/dev/rcBack")]
        self.addresses = [self.get_address("~address_front", "128"),self.get_address("~address_rear", "129")]
        self.baud_rate = int(rospy.get_param("~baud", "460800"))
        self.drivers = self.setup_devices(self.dev_names, self.addresses, self.baud_rate)
        self.read_and_reset_all_devices()
        self.parameters_setup()
        self.publishers_and_subscribers_setup()
        self.pid_and_motor_currents_setup()
        rospy.on_shutdown(self.shutdown)        


    
    def get_address(self, param_name, default_value):
        address = int(rospy.get_param(param_name, default_value))
        if address > self.MAX_ADDRESS or address < self.MIN_ADDRESS:
            rospy.logfatal("Address " + str(address) + " out of range. Must be in range " + str(self.MIN_ADDRESS) + " - " + str(self.MAX_ADDRESS))
            rospy.signal_shutdown("Address out of range")
            return None
        return address

    def get_version_and_log(self, driver):
        version = None
        try:
            version = driver.ReadVersion()
        except Exception as e:
            rospy.logwarn("Problem getting Roboclaw version from address: " + str(driver.address))
            rospy.logdebug(e)
        return version
    
    def setup_devices(self, dev_names, addresses, baud_rate):
        drivers = []
        for dev_name, address in zip(dev_names, addresses):
            try:
                driver = RoboclawDriver(dev_name, baud_rate, address)
                drivers.append(driver)
            except Exception as e:
                rospy.logfatal(f"Could not connect to Roboclaw at {dev_name} with address {str(address)}")
                rospy.logdebug(e)
                rospy.signal_shutdown("Could not connect to Roboclaw")
                return

            version = self.get_version_and_log(driver)
            if version is not None:
                if not version[0]:
                    rospy.logwarn(f"Could not get version from Roboclaw at address: {str(address)}")
                else:
                    rospy.logdebug(repr(version[1]))

        return drivers

    def read_and_reset_all_devices(self):
        if self.drivers is not None:
            for driver, address in zip(self.drivers, self.addresses):
                driver.SpeedM1M2(address, 0, 0)
                driver.ResetEncoders(address)
                
                
    def parameters_setup(self):
        self.MAX_SPEED = self.get_param("~max_speed", 2.0, float)
        self.TICKS_PER_ROTATION = self.get_param("~ticks_per_rotation", 2000, float)
        self.GEAR_RATIO = self.get_param("~gear_ratio", 32, float)
        self.WHEEL_CIRC = self.get_param("~wheel_circ", 0.964, float)
        self.BASE_WIDTH = self.get_param("~base_width", 0.315, float)
        self.PUB_ODOM = self.get_param("~pub_odom", "true", str) == 'true'
        self.PUB_CURRENTS = self.get_param("~pub_currents", "false", str) == 'false'
        self.STOP_MOVEMENT = self.get_param("~stop_movement", "true", str) == 'true'
        self.MAX_ACCEL = self.get_param("~max_accel", 1.0, float)
        self.P = self.get_param("~p_constant", 0.05, float)
        self.I = self.get_param("~i_constant", 0.02, float)
        self.D = self.get_param("~d_constant", 0.0, float)
        self.TICKS_PER_METER = self.TICKS_PER_ROTATION * self.GEAR_RATIO / self.WHEEL_CIRC

    def publishers_and_subscribers_setup(self):
        self.encodm = None
        if (self.PUB_ODOM):
            self.encodm = EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH)
        self.movement = Movement(self.drivers, self.addresses, self.MAX_SPEED, self.BASE_WIDTH, self.TICKS_PER_METER, self.MAX_ACCEL)
        self.last_set_speed_time = rospy.get_rostime()

        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)
        self.current_pub = rospy.Publisher(rospy.get_param('~name')+"/currents", Float32MultiArray, queue_size=10)

    def pid_and_motor_currents_setup(self):
        # PID settings
        # self.rc_front.SetM1VelocityPID(self.P, self.I, self.D, 150000)
        # self.rc_front.SetM2VelocityPID(self.P, self.I, self.D, 150000)
        # self.rc_rear.SetM1VelocityPID(self.P, self.I, self.D, 150000)
        # self.rc_rear.SetM2VelocityPID(self.P, self.I, self.D, 150000)

        # Set max motor currents
        if self.drivers is not None:
            for driver, address in zip(self.drivers, self.addresses):
                driver.SetM1MaxCurrent(address, 5000)
                driver.SetM2MaxCurrent(address, 5000)
                driver.SetMainVoltages(address, 150, 280)
                rospy.sleep(1)
        else:
            raise ValueError("Motor controller addresses are not set.")

    def get_param(self, name, default, type):
        try:
            return type(rospy.get_param(name, default))
        except ValueError:
            rospy.logwarn("Invalid type for parameter " + name + ". Using default value.")
            return default

    def run(self):
        rospy.loginfo("Starting motor drive")
        r_time = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.drivers is not None:
                for driver, address in zip(self.drivers, self.addresses):
                    # stop movement if robot doesn't recieve commands for 1 sec
                    if (self.STOP_MOVEMENT and not self.movement.stopped and rospy.get_rostime().to_sec() - self.movement.last_set_speed_time.to_sec() > 1):
                        rospy.loginfo("Did not get command for 1 second, stopping")
                        try:
                            driver.ForwardM1(address, 0)
                            driver.ForwardM2(address, 0)
                        except OSError as e:
                            rospy.logerr("Could not stop")
                            rospy.logdebug(e)
                    self.movement.stopped = True

            # TODO need find solution to the OSError11 looks like sync problem with serial
            status_front_right, enc_front_right, _ = None, None, None
            status_front_left, enc_front_left, _ = None, None, None
            status_rear_right, enc_rear_right, _ = None, None, None
            status_rear_left, enc_rear_left, _ = None, None, None

            
            if self.drivers is not None:
                try:
                    status_front_right, enc_front_right, _ = self.drivers[0].ReadEncM1(self.addresses[0])
                except ValueError:
                    pass
                except OSError as e:
                    rospy.logwarn("ReadEncM1 front OSError: %d", e.errno)
                    rospy.logdebug(e)

                try:
                    status_front_left, enc_front_left, _ = self.drivers[0].ReadEncM2(self.addresses[0])
                except ValueError:
                    pass
                except OSError as e:
                    rospy.logwarn("ReadEncM2 front OSError: %d", e.errno)
                    rospy.logdebug(e)

                try:
                    status_rear_right, enc_rear_right, _ = self.drivers[1].ReadEncM1(self.addresses[1])
                except ValueError:
                    pass
                except OSError as e:
                    rospy.logwarn("ReadEncM1 rear OSError: %d", e.errno)
                    rospy.logdebug(e)

                try:
                    status_rear_left, enc_rear_left, _ = self.drivers[1].ReadEncM2(self.addresses[1])
                except ValueError:
                    pass
                except OSError as e:
                    rospy.logwarn("ReadEncM2 rear OSError: %d", e.errno)
                    rospy.logdebug(e)

                if (status_front_right is not None and status_front_left is not None and status_rear_right is not None and status_rear_left is not None):
                    rospy.logdebug("Encoders %d %d %d %d" % (enc_front_left, enc_front_right, enc_rear_left, enc_rear_right))
                    if (self.encodm):
                        self.encodm.update_publish(enc_front_left, enc_front_right, enc_rear_left, enc_rear_right)

            self.movement.run()

            # Publish motor currents
            if (self.PUB_CURRENTS):
                if self.drivers is not None and self.addresses is not None:
                    for driver, address in zip(self.drivers, self.addresses):
                        _, cur1, cur2 = driver.ReadCurrents(address)
                        rospy.loginfo("currents : %d %d" % (cur1, cur2))
                        currents = Float32MultiArray(data=[cur1/100.0, cur2/100.0])
                        self.current_pub.publish(currents)
        r_time.sleep()

    def cmd_vel_callback(self, twist):
        self.movement.last_set_speed_time = rospy.get_rostime()
        self.movement.twist = twist

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        rospy.loginfo("Shutting down")
        if self.drivers is not None:
            for driver, address in zip(self.drivers, self.addresses):
                try:
                    driver.ForwardM1(address, 0)
                    driver.ForwardM2(address, 0)
                except OSError as e:
                    rospy.logerr("Shutdown did not work trying again")
                    rospy.logdebug(e)
                    try:
                        driver.ForwardM1(address, 0)
                        driver.ForwardM2(address, 0)
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



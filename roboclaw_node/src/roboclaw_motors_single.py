#!/usr/bin/env python
from math import pi, cos, sin

from driver.roboclaw_driver import RoboclawDriver
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry


# TODO need to find some better was of handling OSerror 11 or preventing it, any ideas?

class EncoderOdom:

    def __init__(self, ticks_per_meter, base_width, publish_tf):
        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = base_width
        self.PUBLISH_TF = publish_tf
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_time = rospy.Time.now()

    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, speed_left, speed_right):

        # Get time elapsed since last measurement
        current_time = rospy.Time.now()
        delta_time = (current_time - self.last_enc_time).to_sec()
        self.last_enc_time = current_time
        
        # Calculate the wheel speeds in m/s
        speed_left_ms = speed_left / self.TICKS_PER_METER
        speed_right_ms = speed_right / self.TICKS_PER_METER

        # Calculate the linear and angular speeds
        vel_x = (speed_right + speed_left) / 2.0 if abs(delta_time) > 0.000001 else 0.0
        vel_theta = (speed_left_ms - speed_right_ms) / self.BASE_WIDTH if abs(delta_time) > 0.000001 else 0.0

        # Calculate the position and orientation
        self.cur_x += vel_x * delta_time * cos(self.cur_theta)
        self.cur_y += vel_x * delta_time * sin(self.cur_theta)
        self.cur_theta = self.normalize_angle(self.cur_theta + vel_theta * delta_time )

        # Debugging
        rospy.logdebug("Speed left " + str(speed_left_ms) + " / Speed right " + str(speed_right_ms) + " / dt " + str(delta_time))
        rospy.logdebug("X: " + str(self.cur_x) + " / Y: " + str(self.cur_y) + " / Theta: " + str(self.cur_theta))
        
        return vel_x, vel_theta

    def update_publish(self, speed_left, speed_right):
        vel_x, vel_theta = self.update(speed_left, speed_right)
        self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
        current_time = rospy.Time.now()

        if (self.PUBLISH_TF):
            br = tf.TransformBroadcaster()
            br.sendTransform(
                (cur_x, cur_y, 0),
                tf.transformations.quaternion_from_euler(0, 0, cur_theta),
                current_time,
                "base_link",
                "odom",
            )

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

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

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)


class Movement:
    def __init__(
        self, driver, max_speed, base_width, ticks_per_meter, max_accel):
        self.twist = None
        self.driver = driver
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

        if self.twist.linear.x != 0 or self.twist.angular.z != 0:
            self.stopped = False

        linear_x = self.twist.linear.x
        linear_x = max(-self.MAX_SPEED, min(linear_x, self.MAX_SPEED))

        # Maybe BASE_WIDTH/2 for skid-steer ? 
        vr = linear_x - self.twist.angular.z * self.BASE_WIDTH  # m/s
        vl = linear_x + self.twist.angular.z * self.BASE_WIDTH
        vr_ticks = int(vr * self.TICKS_PER_METER)  # ticks/s
        vl_ticks = int(vl * self.TICKS_PER_METER)

        self.twist = None
        rospy.logdebug("vr_ticks:%d vl_ticks: %d", vr_ticks, vl_ticks)

        try:
            self.vr_ticks = vr_ticks
            self.vl_ticks = vl_ticks
            self.driver.SpeedM1M2(int(self.vr_ticks), int(self.vl_ticks))
        except OSError as e:
            rospy.logwarn("SpeedM1M2 OSError: %d", e.errno)
            rospy.logdebug(e)


class Node:

    MAX_ADDRESS = 0x87
    MIN_ADDRESS = 0x80

    def __init__(self):
        rospy.init_node(rospy.get_param("~name", "roboclaw_motors"))
        self.dev_name = rospy.get_param("~dev", "/dev/rcMotors")
        self.address = self.get_address("~address", "128")
        self.baud_rate = int(rospy.get_param("~baud", "115200"))
        self.driver = self.setup_device(self.dev_name, self.address, self.baud_rate)

        self.reset_device()
        self.parameters_setup()
        self.publishers_and_subscribers_setup()
        #self.pid_and_motor_currents_setup()
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
        self.driver.StopMotors()
        self.driver.ResetEncoders()

    def parameters_setup(self):
        self.RATE = self.get_param("~rate", 20, int)
        self.MAX_SPEED = self.get_param("~max_speed", 2.0, float)
        self.TICKS_PER_ROTATION = self.get_param("~ticks_per_rotation", 2000, float)
        self.GEAR_RATIO = self.get_param("~gear_ratio", 32, float)
        self.WHEEL_CIRC = self.get_param("~wheel_circ", 0.964, float)
        self.BASE_WIDTH = self.get_param("~base_width", 0.315, float)
        self.PUBLISH_ODOM = self.get_param("~publish_odom", True, bool)
        self.PUBLISH_TF = self.get_param("~publish_tf", False, bool)
        self.PUBLISH_CURRENTS = self.get_param("~publish_currents", False, bool)
        self.STOP_MOVEMENT = self.get_param("~stop_movement", True, bool)
        self.MAX_ACCEL = self.get_param("~max_accel", 1.0, float)
        self.P = self.get_param("~p_constant", 0.05, float)
        self.I = self.get_param("~i_constant", 0.02, float)
        self.D = self.get_param("~d_constant", 0.0, float)
        self.TICKS_PER_METER = (
            self.TICKS_PER_ROTATION * self.GEAR_RATIO / self.WHEEL_CIRC
        )

    def publishers_and_subscribers_setup(self):
        self.encodm = None
        if self.PUBLISH_ODOM:
            self.encodm = EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH, self.PUBLISH_TF)
        self.movement = Movement(
            self.driver,
            self.MAX_SPEED,
            self.BASE_WIDTH,
            self.TICKS_PER_METER,
            self.MAX_ACCEL,
        )
        self.last_set_speed_time = rospy.get_rostime()

        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)
        self.current_pub = rospy.Publisher(
            rospy.get_param("~name") + "/currents", Float32MultiArray, queue_size=1
        )

    def pid_and_motor_currents_setup(self):
        # TODO: NOT USED, NEED TO TEST AS IT DID NOT WORK
        # PID settings
        self.driver.SetM1VelocityPID(self.P, self.I, self.D, 150000)
        self.driver.SetM2VelocityPID(self.P, self.I, self.D, 150000)

        # Set max motor currents
        self.driver.SetM1MaxCurrent(5000)
        self.driver.SetM2MaxCurrent(5000)
        self.driver.SetMainVoltages(150, 280)
        rospy.sleep(1)

    def get_param(self, name, default, type):
        try:
            return type(rospy.get_param(name, default))
        except ValueError:
            rospy.logwarn(
                "Invalid type for parameter " + name + ". Using default value."
            )
            return default

    def run(self):
        rospy.loginfo("Starting motor drive")
        r_time = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            # stop movement if robot doesn't receive commands for 1 sec
            if (
                self.STOP_MOVEMENT and not self.movement.stopped
                and rospy.get_rostime().to_sec() - self.movement.last_set_speed_time.to_sec() > 1
            ):
                rospy.loginfo("Did not get command for 1 second, stopping")
                try:
                    self.driver.StopMotors()
                    self.movement.stopped = True
                except OSError as e:
                    rospy.logerr("Could not stop")
                    rospy.logdebug(e)

            speed_left, speed_right = None, None
            status_left, status_right = None, None
            try:
                # TODO: Test if GetMotorAverageSpeeds() returns the same thing
                # TODO #2: Validate if we shouldn't use GetISpeedCounters() instead
                status_left, speed_left, _ = self.driver.ReadSpeedM1()
                status_right, speed_right, _ = self.driver.ReadSpeedM2()
            except OSError as e:
                rospy.logwarn("ReadSpeeds OSError: %d", e.errno)
                rospy.logdebug(e)
                
            if (status_left == 1 and status_right == 1):
                rospy.logdebug("Encoders speeds %d %d" % (speed_left, speed_right))
                if self.encodm:
                    self.encodm.update_publish(speed_left, speed_right)

            self.movement.run()

            # Publish motor currents
            if self.PUBLISH_CURRENTS:
                _, cur1, cur2 = self.driver.ReadCurrents()
                rospy.loginfo("currents : %d %d" % (cur1, cur2))
                currents = Float32MultiArray(data=[cur1 / 100.0, cur2 / 100.0])
                self.current_pub.publish(currents)
        r_time.sleep()

    def cmd_vel_callback(self, twist):
        self.movement.last_set_speed_time = rospy.get_rostime()
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

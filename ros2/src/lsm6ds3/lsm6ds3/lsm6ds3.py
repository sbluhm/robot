from .imu_driver.lsm6ds3 import LSM6DS3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Imu
from std_srvs.srv import Trigger
import time
from custom_interfaces.msg import Vector
from diagnostic_msgs.msg import DiagnosticStatus

class ImuDriverROSWrapper(Node):

    def __init__(self):
        super().__init__('lsm6ds3_driver')
        self.declare_parameter('~gyro_range', LSM6DS3.GYR_250_DEG_S);
        self.declare_parameter('~accel_range', LSM6DS3.ACC_2_G);
        self.declare_parameter('~dlpf_bandwidth', LSM6DS3.DLPF_260_HZ);
        self.declare_parameter('~gyro_x_offset', 0.0);
        self.declare_parameter('~gyro_y_offset', 0.0);
        self.declare_parameter('~gyro_z_offset', 0.0);
        self.declare_parameter('~accel_x_offset', 0.0);
        self.declare_parameter('~accel_y_offset', 0.0);
        self.declare_parameter('~accel_z_offset', 0.0);

        self.publisher_ = self.create_publisher(Stringi, 'topic', 10)

        gyroRange = self.get_parameter('~gyro_range').get_parameter_value().integer_value
        accelerometerRange = self.get_parameter('~accel_range').get_parameter_value().integer_value
        publish_ims_frequency = 1.0/self.get_parameter('~gyro_range').get_parameter_value().integer_value
  
        # LPF1_BW_SEL, FTYPE
        dlpfBandwidth = self.get_parameter('~dlpf_bandwidth').get_parameter_value().integer_value
        GyroscopeOffset = (
            self.get_parameter('~gyro_x_offset').get_parameter_value().double_value,
            self.get_parameter('~gyro_y_offset').get_parameter_value().double_value,
            self.get_parameter('~gyro_z_offset').get_parameter_value().double_value
        )
        AccelerometerOffset = (
            self.get_parameter('~accel_x_offset').get_parameter_value().double_value,
            self.get_parameter('~accel_y_offset').get_parameter_value().double_value,
            self.get_parameter('~accel_z_offset').get_parameter_value().double_value
        )

        self.imu_pub = self.create_publisher(Imu, "imu", 10)

        self.imu = LSM6DS3(acc_interrupt=False, gyro_interrupt=False, acc_scale=accelerometerRange, gyro_scale=gyroRange)
        self.timer = self.create_timer(publish_ims_frequency, self.publish_imu)

    def publish_imu(self):
        message = Imu()
# use TIMESTAMP0_REG

        message.header.stamp = time.time()
        message.header.frame_id = "base_link";
        message.linear_acceleration_covariance = [0,0,0];
        message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = LSM6DS3.getAccData();
        message.angular_velocity_covariance[0] = 0;
        message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = LSM6DS3.getGyroData();
# Invalidate quaternion
        message.orientation_covariance[0] = -1;
        message.orientation.x = 0;
        message.orientation.y = 0;
        message.orientation.z = 0;
        message.orientation.w = 0;
        self.imu_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)

    imu_driver_wrapper = ImuDriverROSWrapper()
    rclpy.spin(imu_driver_wrapper)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_driver_wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
 

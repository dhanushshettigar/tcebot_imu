import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3
import board
import adafruit_bno055
import time

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # Initialize the publisher
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)

        # Initialize the timer for periodic publishing (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_imu_data)

        # Initialize the sensor
        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        self.get_logger().info("BNO055 IMU initialized")

        # Wait for IMU to stabilize
        time.sleep(1)

        # Define sensor-specific offsets (Verify if needed)
        self.offsets_gyroscope = (0.0, 0.0, 0.0)  # Adjust if needed
        self.offsets_accelerometer = (0.0, 0.0, 0.0)  # Adjust if needed

        # Set proper covariance values (calibrated for BNO055)
        self.orientation_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
        self.angular_velocity_covariance = [0.002, 0, 0, 0, 0.002, 0, 0, 0, 0.002]
        self.linear_acceleration_covariance = [0.003, 0, 0, 0, 0.003, 0, 0, 0, 0.003]

    def publish_imu_data(self):
        msg = Imu()

        # Populate the header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Read orientation (quaternion)
        quaternion = self.sensor.quaternion
        if quaternion and all(q is not None for q in quaternion) and len(quaternion) == 4:
            msg.orientation = Quaternion(
                x=float(quaternion[1]),
                y=float(quaternion[2]),
                z=float(quaternion[3]),
                w=float(quaternion[0])
            )
            msg.orientation_covariance = self.orientation_covariance
        else:
            self.get_logger().warn("Failed to read quaternion data, skipping...")
            return  # Skip publishing if invalid data

        # Read angular velocity (gyro)
        gyro = self.sensor.gyro
        if gyro and all(g is not None for g in gyro):
            msg.angular_velocity = Vector3(
                x=float(gyro[0] - self.offsets_gyroscope[0]),
                y=float(gyro[1] - self.offsets_gyroscope[1]),
                z=float(gyro[2] - self.offsets_gyroscope[2])
            )
            msg.angular_velocity_covariance = self.angular_velocity_covariance
        else:
            self.get_logger().warn("Failed to read gyroscope data, skipping...")
            return  # Skip publishing if invalid data

        # Read linear acceleration
        accel = self.sensor.linear_acceleration
        if accel and all(a is not None for a in accel):
            msg.linear_acceleration = Vector3(
                x=float(accel[0] - self.offsets_accelerometer[0]),
                y=float(accel[1] - self.offsets_accelerometer[1]),
                z=float(accel[2] - self.offsets_accelerometer[2])
            )
            msg.linear_acceleration_covariance = self.linear_acceleration_covariance
        else:
            self.get_logger().warn("Failed to read linear acceleration data, skipping...")
            return  # Skip publishing if invalid data

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().debug("Published IMU data successfully")

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("IMU publisher stopped cleanly")
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
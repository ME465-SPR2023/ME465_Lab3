import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from transforms3d.euler import quat2euler
from transforms3d.quaternions import qinverse, qmult


class Lab3(Node):
    dt = 0.02
    tag_size = 0.2
    Q = np.diag((0.5, 2.0, 0.001))
    particles = 100

    def __init__(self):
        super().__init__("lab3_node")

        self.create_timer(self.dt, self.timer_callback)
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            5,
        )
        self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            5,
        )
        self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            5,
        )
        self.viz_publisher = self.create_publisher(
            MarkerArray,
            '/viz',
            5,
        )
        self.declare_parameter("map")
        map = self.get_parameter("map").value
        if map is None:
            self.get_logger().warn("map is not defined.")
            self.map = np.ndarray((0, 2))
        else:
            self.map = np.array(map).reshape((-1, 2))
        self.random = np.random.default_rng()
        self.camera_P = None
        self.z = np.ndarray((3, 0))
        self.x = np.array([[-3.5, -1.0, 0.0]]).T * np.ones((3, self.particles))
        self.x += np.array([[2, 2, 2 * np.pi]]).T * self.random.uniform(size=(3, self.particles)) - np.array([[1, 1, np.pi]]).T
        self.w = np.ones((self.particles,)) / self.particles
        self.last_odom = None
        self.odom = None

    @staticmethod
    def odometry2loc(msg):
        return np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ])

    @staticmethod
    def odometry2quat(msg):
        return np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        ])

    def quat_diff(self, a, b):
        if isinstance(a, Odometry):
            a = self.odometry2quat(a)
        if isinstance(b, Odometry):
            b = self.odometry2quat(b)
        return qmult(a, qinverse(b))

    def get_input(self):
        if self.odom is None:
            return 0, 0
        if self.last_odom is None:
            self.last_odom = self.odom
        dtheta = quat2euler(self.quat_diff(self.odom, self.last_odom))[2]
        dx = self.odometry2loc(self.odom) - self.odometry2loc(self.last_odom)
        self.last_odom = self.odom
        return np.linalg.norm(dx) / self.dt, dtheta / self.dt

    def timer_callback(self):
        v, w = self.get_input()
        # Your code goes here.
        self.resample()
        self.z = np.ndarray((3, 0))
        self.publish_estimate(np.sum(self.w * self.x, axis=1))

    def resample(self):
        if np.any(np.isnan(self.w)):
            return
        if self.w.min() / self.w.max() < 0.5:
            ind = tuple(np.random.choice(self.particles, size=(self.particles,), p=self.w))
            self.x = self.x[:, ind]
            self.w = self.w[ind,]

    def publish_estimate(self, mean):
        map = MarkerArray()
        for i in range(self.map.shape[0]):
            marker = Marker()
            marker.id = i
            marker.ns = "map"
            marker.type = 9
            marker.action = 0
            marker.text = str(i)
            marker.scale.z = 0.5
            marker.pose.position.x = self.map[i,0]
            marker.pose.position.y = self.map[i,1]
            marker.pose.position.z = 0.0
            marker.header.frame_id = "odom"
            marker.color.a=1.0
            marker.color.b=1.0
            map.markers.append(marker)
        self.viz_publisher.publish(map)
        robot = Marker()
        robot.id = -1
        robot.ns = "robot"
        robot.type = 0
        robot.action = 0
        robot.scale.x = 0.15
        robot.scale.y = 0.3
        robot.scale.z = 0.0
        theta = mean[2]
        robot.points = [
            Point(x=mean[0], y=mean[1]),
            Point(x=mean[0] + 0.5 * np.cos(theta), y=mean[1] + 0.5 * np.sin(theta)),
        ]
        robot.header.frame_id = "odom"
        robot.color.a = 1.0
        robot.color.g = 1.0
        self.viz_publisher.publish(
            MarkerArray(markers=[robot])
        )
        particles = MarkerArray()
        for i in range(self.x.shape[1]):
            particle = Marker()
            particle.id = i
            particle.ns = "particles"
            particle.type = 0
            particle.action = 0
            particle.scale.x = 0.07
            particle.scale.y = 0.15
            particle.scale.z = 0.0
            x = self.x[:, i]
            particle.points = [
                Point(x=x[0], y=x[1]),
                Point(x=x[0] + 0.25 * np.cos(x[2]), y=x[1] + 0.25 * np.sin(x[2])),
            ]
            particle.header.frame_id = "odom"
            particle.color.a = 1.0
            particle.color.r = 1.0
            particles.markers.append(particle)
        self.viz_publisher.publish(particles)

    def odom_callback(self, msg):
        self.odom = msg

    def camera_info_callback(self, msg):
        self.camera_P = msg.p.reshape((3, 4))[:, :3]

    def detection_callback(self, msg):
        if self.camera_P is None:
            return
        self.z = np.zeros((3, len(msg.detections)))
        for i, detection in enumerate(msg.detections):
            T = np.linalg.inv(self.camera_P) @ detection.homography.reshape((3, 3))
            tt = T[:, 2] / np.linalg.norm(T[:, 0]) * self.tag_size / 2
            self.z[:, i] = (
                tt[2],
                np.arctan2(tt[0], tt[2]),
                detection.id,
            )


def main(args=None):
    rclpy.init(args=args)
    node = Lab3()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

import math
import os
import random
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Trigger

from geometry_msgs.msg import Quaternion
from vortex_msgs.srv import SendWaypoints
from vortex_msgs.msg import Waypoint

from ament_index_python.packages import get_package_share_directory

import subprocess
import signal
import shutil
import os

import logging
from datetime import datetime



class WaypointBagger:

    def __init__(self):
        self.nr = 1
        self.processes = {}

    def start(self, name, topic):
        bag_name = f"{name}{self.nr}"

        proc = subprocess.Popen([
            "ros2",
            "bag",
            "record",
            "-o",
            bag_name,
            topic
        ])

        self.processes[bag_name] = proc
        print(f"Recording started: {bag_name}")

        self.nr += 1
        return bag_name


    def stop(self, bag_name):
        if bag_name in self.processes:
            proc = self.processes[bag_name]
            proc.send_signal(signal.SIGINT)
            proc.wait()

            print(f"Recording stopped: {bag_name}")
            del self.processes[bag_name]


    def delete(self, bag_name):
        if os.path.exists(bag_name):
            shutil.rmtree(bag_name)
            print(f"Bag deleted: {bag_name}")


bagger = WaypointBagger()

def quat_from_yaw(yaw):
    """Convert yaw to quaternion (same as quatFromYaw in C++)"""
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


def quat_to_euler(x, y, z, w, degrees=False):

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    if degrees:
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)

    return [roll, pitch, yaw]


best_effort_qos = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)


# Read goal
package_path = get_package_share_directory('test_pipeline')
file_path = os.path.join(package_path, 'config', 'goal_pose.yaml')

with open(file_path) as f:
    data = yaml.safe_load(f)

goal_pos = data["goal_pos"]
goal_ori = data["goal_ori"]

return_pos = data["return_pos"]
return_ori = data["return_ori"]

pos_tol_goal = data["pos_tol_goal"]
ori_tol_goal = data["ori_tol_goal"]

pos_tol_home = data["pos_tol_home"]
ori_tol_home = data["ori_tol_home"]

timeout = data["timeout"]

bag_configs = data["bags"]  # list of {name, topic}

_rand = data.get("random_start", {})
random_start_enabled  = _rand.get("enabled", False)
random_start_pos      = _rand.get("randomize_pos", True)
random_start_pos_range = _rand.get("pos_range", 0.0)
random_start_ori      = _rand.get("randomize_ori", False)
random_start_ori_range = _rand.get("ori_range", 0.0)


class CheckGoalNode(Node):
    def __init__(self):
        super().__init__('check_goal_node')

        # Compute (possibly randomized) start target for first iteration
        self._apply_random_start()

        # Start recording all configured bags
        self.active_bags = [
            bagger.start(cfg["name"], cfg["topic"]) for cfg in bag_configs
        ]

        self.goal_reached = False
        self.sendWaypoint = False

        self.iteration = 1
        self.last_pose = None

        self.start_time = time.time()

        self.msg = "Runnig test"

        self.pose_sub_ = self.create_subscription(
            PoseWithCovarianceStamped,
            '/orca/pose',
            self.pose_callback,
            best_effort_qos,
        )

        # Service
        self.goal_service = self.create_service(
            Trigger,
            'pipline/goal_service',
            self.goal_service_callback
        )

        # Create client
        self.client = self.create_client(
            SendWaypoints,
            '/orca/waypoint_addition'
        )     

        self.current_pose_ = None
        self.received_pose_ = False

        # Create experiment logger
        log_dir = os.path.expanduser("~/test_logs")
        os.makedirs(log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d")
        log_file = os.path.join(log_dir, f"pipeline_test_{timestamp}.log")

        self.file_logger = logging.getLogger("pipeline_logger")
        self.file_logger.setLevel(logging.INFO)

        handler = logging.FileHandler(log_file)
        formatter = logging.Formatter(
            "%(asctime)s | %(message)s",
            "%Y-%m-%d %H:%M:%S"
        )

        handler.setFormatter(formatter)
        self.file_logger.addHandler(handler)

        self.get_logger().info(f"Logging to file: {log_file}")

    def _apply_random_start(self):
        pos = list(return_pos)
        ori = list(return_ori)  # [x, y, z, w]
        if random_start_enabled:
            if random_start_pos:
                r = random_start_pos_range
                pos[0] += random.uniform(-r, r)
                pos[1] += random.uniform(-r, r)
                pos[2] += random.uniform(-r, r)
            if random_start_ori:
                r = random_start_ori_range
                yaw = quat_to_euler(ori[0], ori[1], ori[2], ori[3])[2]
                yaw += random.uniform(-r, r)
                q = quat_from_yaw(yaw)
                ori = [q.x, q.y, q.z, q.w]
        self.return_target_pos = pos
        self.return_target_ori = ori
        self.get_logger().info(
            f"Next start target: pos={[round(v,3) for v in pos]}, ori={[round(v,3) for v in ori]}"
        )

    def log_iteration(self, result, x, y, z, ori, time):
        self.file_logger.info(
            f"Iteration={self.iteration} | "
            f"Result={result} | "
            f"Position=({x:.3f}, {y:.3f}, {z:.3f}) | "
            f"Orientation=({ori[0]:.3f}, {ori[1]:.3f}, {ori[2]:.3f}) | "
            f"Time since start={time}"
        )

    def send_waypoint(self, x, y, z, w_o, x_o, y_o, z_o, mode,
                      switching_threshold,
                      overwrite_prior,
                      take_priority):

        # Build waypoint
        wp = Waypoint()
        wp.pose.position.x = x
        wp.pose.position.y = y
        wp.pose.position.z = z
        wp.pose.orientation.w = w_o
        wp.pose.orientation.x = x_o
        wp.pose.orientation.y = y_o
        wp.pose.orientation.z = z_o
        wp.mode = mode

        # Build request
        req = SendWaypoints.Request()
        req.waypoints = [wp]
        req.switching_threshold = switching_threshold
        req.overwrite_prior_waypoints = overwrite_prior
        req.take_priority = take_priority

        # Call service
        self.future = self.client.call_async(req)



    def goal_service_callback(self, request, response):
        response.success = self.goal_reached
        response.message = self.msg
        return response


    def pose_callback(self, msg):
        self.current_pose_ = msg
        self.received_pose_ = True

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        q_w = msg.pose.pose.orientation.w
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z

        current_ori = quat_to_euler(x=q_x, y=q_y, z=q_z, w=q_w)

        self.last_pose = (x, y, z, current_ori)

        dist = math.sqrt(
            (goal_pos[0] - x) ** 2 +
            (goal_pos[1] - y) ** 2 +
            (goal_pos[2] - z) ** 2
        )

        dist_ori = math.sqrt(
            (goal_ori[0] - current_ori[0]) ** 2 +
            (goal_ori[1] - current_ori[1]) ** 2 +
            (goal_ori[2] - current_ori[2]) ** 2
        )

        home_dist = math.sqrt(
            (self.return_target_pos[0] - x) ** 2 +
            (self.return_target_pos[1] - y) ** 2 +
            (self.return_target_pos[2] - z) ** 2
        )

        home_ori = math.sqrt(
            (self.return_target_ori[0] - current_ori[0]) ** 2 +
            (self.return_target_ori[1] - current_ori[1]) ** 2 +
            (self.return_target_ori[2] - current_ori[2]) ** 2
        )
        elapsed = time.time() - self.start_time
        if self.goal_reached == False:
            if dist < pos_tol_goal and dist_ori < ori_tol_goal:
                for bag in self.active_bags:
                    bagger.stop(bag)
                    bagger.delete(bag)
                self.get_logger().info(
                        "goal reached"
                    )
                self.goal_reached = True
                self.msg = "goal reached and returning to start"
                self.sendWaypoint = True

                self.get_logger().info(
                    f"Drone reached goal: {goal_pos}, {goal_ori}"
                )

                self.get_logger().info(
                    f"Final pose: ({x}, {y}, {z}) {current_ori}"
                )
                self.log_iteration("GOAL_REACHED", x, y, z, current_ori, elapsed)
            
            if elapsed >= timeout:
                for bag in self.active_bags:
                    bagger.stop(bag)
                self.msg = "timeout returning to start"
                self.goal_reached = True
                self.sendWaypoint = True
                self.get_logger().info("timeout")
                self.log_iteration("TIMEOUT", x, y, z, current_ori, elapsed)


        elif self.goal_reached == True:
            if self.sendWaypoint == True:
                self.sendWaypoint = False
                self.iteration += 1
                self.send_waypoint(
                    x=self.return_target_pos[0],
                    y=self.return_target_pos[1],
                    z=self.return_target_pos[2],
                    w_o=self.return_target_ori[3],
                    x_o=self.return_target_ori[0],
                    y_o=self.return_target_ori[1],
                    z_o=self.return_target_ori[2],
                    mode=0,
                    switching_threshold=0.5,
                    overwrite_prior=True,
                    take_priority=True
                )
                          
            if home_dist < pos_tol_home and home_ori < ori_tol_home:
                    self._apply_random_start()
                    self.active_bags = [
                        bagger.start(cfg["name"], cfg["topic"]) for cfg in bag_configs
                    ]
                    self.goal_reached = False
                    self.msg = "Runnig test"
                    self.start_time = time.time()

                    self.get_logger().info(
                        "Drone retuned to start"
                    )

def main(args=None):
    rclpy.init(args=args)

    node = CheckGoalNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
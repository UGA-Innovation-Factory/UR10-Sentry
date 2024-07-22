from Robot.UR.URRobot import URRobot
from time import sleep
import math
import threading


class URSentry:
    def __init__(self, host):
        self.robot = URRobot(host)
        self.sentry_pose = [-0.0, -2.094, 0.96, -0.436, -1.571, 1.318]
        self.forward_pose = [1.571, -1.949, 1.974, -2.548, -1.571, 1.318]
        self.imposing_pose = [1.571, -1.41, 1.411, -2.754, -1.604, 1.406]
        self.robot_speed = [0, 0, 0, 0, 0, 0]
        self.detections = []

        # Flags
        self.await_stop = False
        self.await_stop_ticks = 0

        self.send_to_zero_on_stop = False

        self.has_detected_once = False

        self.none_input_ticks = 0
        self.is_on_sentry_mode = True
        self.has_initialized = False

        # Smooth stopping
        self.smooth_stop_delayed_call = None

    def initialize_pose(self):
        self.await_stop = True
        self.sentry_position()

    def get_joint_angles(self) -> "list[float]":
        """
        Get the current joint angles of the robot in degrees
        """
        return self.robot.get_joint_angles_degrees()

    def sentry_position(self, a=0.5, v=1.5):
        """
        Return the robot to the sentry position
        """
        self.robot.movej(self.sentry_pose, a=0.5, v=1.5)

    def forward_position(self, a=0.5, v=1.5):
        self.robot.movej(self.imposing_pose, a, v)

    def update_detections(self, detections):
        self.detections = detections

    def forward_position_to_base_angle_degrees(self, base_angle, a=0.5, v=1.5):
        pose = self.imposing_pose
        pose[0] = math.radians(base_angle)
        self.robot.movej(pose, a, v)

    def set_base_speed(self, speed):
        self.robot_speed[0] = speed

    def set_neck_speed(self, speed):
        self.robot_speed[4] = speed

    def smooth_stop(self):
        # print("Smooth stopping --------------------")
        self.robot.speedj([0, 0, 0, 0, 0, 0], 1.5)

    def lerp(self, a, b, t):
        return a + t * (b - a)


    def control_robot(self, joystick_pos: list[float] | None):
        """
        Control the robot based on a joystick input.
        """

        # Check for flags that would block control due to things happening

        # Initialize robot if it hasn't already
        if not self.has_initialized:
            self.initialize_pose()
            self.has_initialized = True
            return

        # If we are awaiting a stop, we do not control the robot until it remains at a standstill for a certain amount of ticks
        ticks_to_wait = 3
        if self.await_stop:
            joint_speeds = self.robot.get_joint_speeds()
            print("Awaiting stop -------------------- speeds: ", joint_speeds)
            if all([speed == 0 for speed in joint_speeds]):
                self.await_stop_ticks += 1
                if self.await_stop_ticks >= ticks_to_wait:
                    self.await_stop = False
                    self.await_stop_ticks = 0
            return

        # If we are awaiting a stop to send to zero, we do so right after it passes the await_stop check
        if self.send_to_zero_on_stop:
            print("Sending to zero --------------------")
            self.await_stop = True
            self.send_to_zero_on_stop = False
            self.has_detected_once = False
            self.forward_position_to_base_angle_degrees(0)
            return

        # If the joystick is None, we return to sentry mode after a certain amount of ticks
        # We only do this if we have detected something at least once, as the camera gets buggy after fast movements
        ticks_for_return_to_sentry = 50
        if joystick_pos is None:
            if self.has_detected_once and not self.is_on_sentry_mode:
                self.none_input_ticks += 1
                print("None input ticks: ", self.none_input_ticks, " / ", ticks_for_return_to_sentry)
                if self.none_input_ticks > ticks_for_return_to_sentry:
                    self.none_input_ticks = 0
                    self.is_on_sentry_mode = True
                    self.await_stop = True
                    self.sentry_position()
            return

        # If we have detected something, we reset the none input ticks

        # If all flags are clear, we can control the robot
        if self.is_on_sentry_mode:
            self.awake_from_sentry_mode(joystick_pos[0], joystick_pos[1])
            self.has_detected_once = False
        else:
            # is on forward mode
            self.move_robot_base(joystick_pos[0], joystick_pos[1])
            self.has_detected_once = True
            self.none_input_ticks = 0


    def awake_from_sentry_mode(self, joystick_pos_x: float, joystick_pos_y: float):
        if joystick_pos_x == 0 and joystick_pos_y == 0:
            return

        theta_rad = math.atan2(joystick_pos_x, -joystick_pos_y)
        theta_deg = math.degrees(theta_rad)
        print("Theta: ", theta_deg)

        self.await_stop = True
        self.forward_position_to_base_angle_degrees(-theta_deg)
        self.is_on_sentry_mode = False
        

    def move_robot_base(self, joystick_pos_x: float, joystick_pos_y: float = 0):
        """
        Move the robot based on a joystick input,
        where the the center is (0, 0) and the bottom right corner is (1, 1).

        Horizontally, the speed of the base is adjusted on how far left or right the detection is.
        We take into account the maximum rotation limit, trying to slow down when nearing +-360 degrees,
        and trying to never reach that point. If needed, the robot should attempt to do a full rotation the other way to keep following
        the target.
        This can be changed so we first change the horizontal "neck" (wrist_2) until it reaches a limit, and then rotate the base,
        which would result in a more natural movement

        Vertically, the speed of the vertical "neck" (wrist_1) is adjusted based on how far up or down the detection is.
        We limit it's rotation to +- 45 degrees, as we do not need more to follow the target.
        This can be changed in the future to acomplish more natural movements, by also adjusting the speed of the shoulder joint.

        Movement is done entirely through speed control, which should result in a more fluid movement compared to pose control.

        If flag await_stop is set True, we check if the current speed is 0 wait until all joints are stopped before moving again.

        """

        joystick_pos_x = -joystick_pos_x
        # We omit all processing until the robot has stopped moving for a certain amount of calls ('ticks')

        # print("Base Speed: ", self.robot_speed[0])


        # Cancel smooth stopping due to no input
        if self.smooth_stop_delayed_call is not None:
            self.smooth_stop_delayed_call.cancel()
            self.smooth_stop_delayed_call = None

        # # Center detections, so that the center of the image is (0, 0)
        # # We also invert the y axis, so that the upper part of the image is positive
        # # Thus: lower left corner is (-500, -500) and upper right corner is (500, 500)
        # joystick_pos_x -= 500
        # joystick_pos_y -= 500
        # joystick_pos_y = -joystick_pos_y

        # Deadzones where we still consider the target in the middle, to avoid jittering
        horizontal_dead_zone_radius = 0.1
        vertical_dead_zone_radius = 0.1

        # Speeds for the base and neck joints
        base_max_speed = 1.5
        base_min_speed = 0.1

        base_acceleration = 1.5

        # Maximum and minimum angles
        base_max_angle = 315
        base_danger_angle = 340

        if abs(joystick_pos_x) < horizontal_dead_zone_radius:
            # We are in the deadzone, so we do not need to move the base, or stop moving it
            if self.robot_speed[0] == 0:
                return
            self.set_base_speed(0)

        else:
            current_pose = self.get_joint_angles()
            base_angle = current_pose[0]
            # print("Base angle: ", base_angle)
            # Check if we are past the maximum angle
            if abs(base_angle) > base_max_angle:
                # We are past the maximum angle, so we undo a rotation by sending to 0
                self.send_to_zero_on_stop = True
                self.await_stop = True
                self.set_base_speed(0)
                self.robot.speedj(self.robot_speed, base_acceleration, 1)
                return

            # We are not in the deadzone, so we need to move the base
            # Calculate the speed based on the distance from the center
            direction = math.copysign(1, joystick_pos_x)
            base_speed = (
                self.lerp(base_min_speed, base_max_speed, abs(joystick_pos_x))
                * direction
            )
            self.set_base_speed(base_speed)

            # Schedule smooth stop if no input is given for a second
            self.smooth_stop_delayed_call = threading.Timer(
                0.4, self.move_robot_base, args=[0]
            )
            self.smooth_stop_delayed_call.start()

        #print("Base speed: ", self.robot_speed[0], " Joystick: ", joystick_pos_x)
        self.robot.speedj(self.robot_speed, base_acceleration, 1)


if __name__ == "__main__":
    host = "172.22.114.160"
    sentry = URSentry(host)
    #sentry.sentry_position()
    # sleep(3)
    # sentry.forward_position()
    # sleep(3)
    # sentry.robot.speedj([0.8, 0, 0, 0, 0, 0], 0.5, 3)
    # sleep(2)
    # sentry.smooth_stop()
    # sleep(2)
    # sentry.forward_position_to_base_angle_degrees(45)

    print(sentry.robot.get_joint_angles())

    # while True:
    #     print(sentry.robot.get_joint_speeds()[0])

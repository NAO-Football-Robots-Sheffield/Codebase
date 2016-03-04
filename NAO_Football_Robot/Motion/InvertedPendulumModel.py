import RobotConstants
import almath
import math
from naoqi import ALProxy
import motion


class LIPM:
    # initialisation of other global parameters; needed here because good practice...
    ending_time = 0
    next_beginning_time = 0
    ZMP_forwards_velocity = 0

    def __init__(self, proxy, beginning_time, curr_step_size, previous_step_size, support_foot):
        self.proxy = proxy
        self.pendulum_height = proxy.getCOM("Body", motion.FRAME_ROBOT, True)[2] * RobotConstants.TO_MILLIMETERS
        self.k = math.sqrt(RobotConstants.GRAVITATIONAL_ACCELERATION / self.pendulum_height)
        self.beginning_time = beginning_time
        self.current_step_size = curr_step_size  # *NEXT* s; 2D vector; s[0] = *NEXT* sx, s[1] = *NEXT* sy s[2] = 0
        self.previous_step_size = previous_step_size
        # (x0)y
        initial_y_position = - proxy.getPosition(support_foot, motion.FRAME_TORSO, True)[1] + \
            proxy.getCOM("Body", motion.FRAME_TORSO, True)[1] * RobotConstants.TO_MILLIMETERS
        self.ZMP_lateral_position = initial_y_position / (math.cosh(self.k * beginning_time))
        self.r = [RobotConstants.REF_PENDULUM_PIVOT_POINT_X, RobotConstants.LEG_DISTANCE/2]

    def compute_next_support_phase_time(self):
        # take a guess of te
        self.ending_time = -self.beginning_time
        offset = 1
        while math.fabs(offset) > 0.0001:
            self.next_beginning_time = (1 / self.k) * math.asinh(
                (self.ZMP_lateral_position * math.sinh(self.k * self.ending_time)) / (-self.ZMP_lateral_position)
            )
            y = self.ZMP_lateral_position * math.cosh(self.k * self.ending_time) - \
                (-self.ZMP_lateral_position) * math.cosh(self.k * self.next_beginning_time)

            distance_between_pendulums = 2 * self.r[1] + self.current_step_size[1]

            ending_time_velocity = self.ZMP_lateral_position * self.k * math.sinh(self.k * self.ending_time)
            offset = (distance_between_pendulums - y) / (2 * ending_time_velocity)
            self.ending_time += offset
        self.next_beginning_time = (1 / self.k) * math.asinh(
            (self.ZMP_lateral_position * math.sinh(self.k * self.ending_time)) / (-self.ZMP_lateral_position)
        )

    def compute_zmp_forwards_velocity(self):
        self.ZMP_forwards_velocity = (self.k * (2 * self.r[0] + self.current_step_size[0])) / (
            math.sinh(self.k * self.ending_time) - math.cosh(self.k * self.ending_time) *
            math.tanh(self.k * self.next_beginning_time)
        )

    def solve_pendulum_problem(self):
        self.compute_next_support_phase_time()
        self.compute_zmp_forwards_velocity()

    def get_feet_positions(self, time):
        feet_positions = {"supporting foot": [- self.get_position_x_component(time),
                                              - self.get_position_y_component(time),
                                              -self.pendulum_height], "non-supporting foot": [0, 0, 0]}
        q = self.get_non_supporting_foot_function(time)
        feet_positions["non-supporting foot"][0] = feet_positions["supporting foot"][0] + \
            (2 * self.r[0] + self.current_step_size[0]) * q - \
            (self.previous_step_size[0] - 2 * self.r[0]) * (1-q)
        feet_positions["non-supporting foot"][1] = feet_positions["supporting foot"][1] + \
            (2 * self.r[1] + self.current_step_size[1]) * q - \
            (self.previous_step_size[1] - 2 * self.r[1]) * (1-q)
        feet_positions["non-supporting foot"][2] = self.get_lifting_function(time)
        return feet_positions

    def get_position_x_component(self, time):
        return self.ZMP_forwards_velocity * (1 / self.k) * math.sinh(self.k * time)

    def get_position_y_component(self, time):
        return self.ZMP_lateral_position * math.cosh(self.k * time)

    def get_phase(self, time):
        return (time - self.beginning_time) / (self.ending_time - self.beginning_time)

    def get_non_supporting_foot_function(self, time):
        return (1 - math.cos(almath.PI * self.get_phase(time))) / 2.0

    def get_lifting_function(self, time):
        lift_multiplier = 30  # Maximum foot height
        return math.sin(self.get_phase(time)) * lift_multiplier

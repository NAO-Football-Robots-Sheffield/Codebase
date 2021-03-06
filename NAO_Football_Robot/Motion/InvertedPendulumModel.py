import RobotConstants.GRAVITATIONAL_ACCELERATION as G
import RobotConstants.LEG_DISTANCE as LEG_DIST
import almath
import math
from naoqi import ALProxy
import motion


class LIPM:
    # TODO: assign a sensible value for this - "arbitrary param" - how?
    ZMP_lateral_position = 0  # (x0)y
    # initialisation of other global parameters; needed here because good practice...
    ending_time = 0
    next_beginning_time = 0
    ZMP_forwards_velocity = 0

    def __init__(self, proxy, beginning_time, curr_step_size):
        self.proxy = proxy
        # Is it necessary to make this a field? YES! p(s) = (-(x(t)x, -(x(t))y, -h)!!!
        self.pendulum_height = proxy.getCOM("Body", motion.FRAME_ROBOT, True)[2]
        self.k = math.sqrt(G / self.pendulum_height)
        self.beginning_time = beginning_time

        #  TODO: Move r constants in RobotConstants?
        self.r_lateral = LEG_DIST / 2.0  # ry
        #  TODO: How do I initialise this?
        self.r_walking_axis = 0  # rx
        self.current_step_size = curr_step_size  # *NEXT* s; 2D vector; s[0] = *NEXT* sx, s[1] = *NEXT* sy

    def get_next_support_phase_time(self):
        # take a guess of te
        self.ending_time = -self.beginning_time
        offset = 1
        while math.fabs(offset) > 0.0001:
            self.next_beginning_time = (1 / self.k) * math.asinh(
                (self.ZMP_lateral_position * math.sinh(self.k * self.ending_time)) / (-self.ZMP_lateral_position)
            )
            y = self.ZMP_lateral_position*math.cosh(self.k*self.ending_time) - \
                (-self.ZMP_lateral_position)*math.cosh(self.k*self.next_beginning_time)

            distance_between_pendulums = 2*self.r_lateral + self.current_step_size[1]

            ending_time_velocity = self.ZMP_lateral_position * self.k * math.sinh(self.k*self.ending_time)
            offset = (distance_between_pendulums - y )/ (2 * ending_time_velocity)
            self.ending_time += offset
        self.next_beginning_time = (1 / self.k) * math.asinh(
                (self.ZMP_lateral_position * math.sinh(self.k * self.ending_time)) / (-self.ZMP_lateral_position)
            )

    def get_x_axis_component(self):
        self.ZMP_forwards_velocity = (self.k * (2*self.r_walking_axis + self.current_step_size[0])) / (
            math.sinh(self.k * self.ending_time) - math.cosh(self.k * self.ending_time) *
            math.tanh(self.k * self.next_beginning_time)
        )

    def get_phase(self, time):
        return (time - self.beginning_time) / (self.ending_time - self.beginning_time)

    def get_non_supporting_foot_function(self, time):
        return (1 - math.cos(almath.PI*self.get_phase(time))) / 2.0

    # TODO: Define lifting function
    def get_lifting_function(self, time):
        pass

    def get_feet_positions(self, time):
        pass

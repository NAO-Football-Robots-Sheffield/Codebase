import almath
import RobotConstants as robot
import math
import motion


class InverseKinematics:

    def __init__(self, proxy):
        self.proxy = proxy

    @staticmethod
    def generate_joint_angles(foot_position, foot):
        if foot.lower not in ["l", "r"]:
            print "ERROR: Foot not specified correctly. Expected \"l\"(for left foot) or \"r\"(for right foot)." + \
                  "\nSpecified foot position: " + foot_position + "\nSpecified foot: " + foot
            pass
        angles = {}
        hip_orthogonal_to_foot = almath.Transform.inverse(
            almath.Transform.fromRotX(almath.PI/4) *
            almath.Transform([0, robot.LEG_DISTANCE, 0]) *
            almath.Transform.fromPosition(foot_position[0],
                                          foot_position[1],
                                          foot_position[2])
            )

        hip_to_foot_distance = hip_orthogonal_to_foot.norm()

        angles[foot.upper()+"KneePitch"] = almath.PI - (math.pow(robot.UPPER_LEG, 2) + math.pow(robot.LOWER_LEG, 2) -
                                                        math.pow(hip_to_foot_distance, 2)) / \
                                                       (2*robot.UPPER_LEG*robot.LOWER_LEG)

        foot_to_hip_orthogonal = hip_orthogonal_to_foot.inverse()
        x = foot_to_hip_orthogonal[3]
        y = foot_to_hip_orthogonal[7]
        z = foot_to_hip_orthogonal[11]

        ankle_pitch1 = math.acos((math.pow(robot.LOWER_LEG, 2) + math.pow(hip_to_foot_distance, 2) -
                                  math.pow(robot.UPPER_LEG, 2)) /
                                 (2*robot.LOWER_LEG*hip_to_foot_distance))
        ankle_pitch2 = math.atan2(x, math.sqrt(math.pow(y, 2) + math.pow(z, 2)))

        angles[foot.upper()+"AnklePitch"] = ankle_pitch1 + ankle_pitch2
        angles[foot.upper() + "AnkleRoll"] = math.atan2(y, z)

        thigh_to_foot = almath.Transform.fromRotX(angles[foot.upper() + "AnkleRoll"]) * \
            almath.Transform.fromRotY(angles[foot.upper() + "AnklePitch"]) * \
            almath.Transform.fromPosition(0, 0, robot.LOWER_LEG) * \
            almath.Transform.fromRotY(angles[foot.upper() + "KneePitch"]) * \
            almath.Transform.fromPosition(0, 0, robot.UPPER_LEG)
        hip_orthogonal_to_thigh = thigh_to_foot.inverse() * hip_orthogonal_to_foot
        angles[foot.upper() + "HipRoll"] = math.asin(hip_orthogonal_to_thigh[9]) - almath.PI/4
        angles[foot.upper() + "HipPitch"] = math.atan2(-hip_orthogonal_to_thigh[8], hip_orthogonal_to_thigh[10])
        angles[foot.upper() + "HipYaw"] = math.atan2(-hip_orthogonal_to_thigh[1], hip_orthogonal_to_thigh[5])
        return angles

    def com_to_torso_reference_change(self, position):
        com_pos = self.proxy.getCOM("Body", motion.FRAME_TORSO, True)
        torso_transform = almath.Transform.fromPosition(position[0], position[1], position[2]) * \
            almath.Transform.fromPosition(com_pos[0], com_pos[1], com_pos[2])
        return [torso_transform[3], torso_transform[7], torso_transform[11]]
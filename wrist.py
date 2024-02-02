import math

# NOTE:
# - wrist range constraint -15 degrees to 65 degrees
# - All degrees right now except for very end where its converted to rads for cos calculation
# - All inches ecept for end when turned into centimeters

ROT_TO_LEAD_CM_RATIO =  20.0 * (1.0 / 2.54)

def rotsToCm(rots):
    return rots / ROT_TO_LEAD_CM_RATIO
def cmToRots(cm):
    return cm * ROT_TO_LEAD_CM_RATIO
def toRads(rads):
    return (rads * math.pi) / 180

WRIST_DEGREES = 65
WRIST_ANGLE_OFFSET = 38.65
BETA = WRIST_DEGREES - WRIST_ANGLE_OFFSET
MOTOR_PIVOT_TO_WRIST_PIVOT = 3.393
WRIST_PIVOT_TO_NUT = 1.566
LEAD_SCREW_LENGTH = math.sqrt(MOTOR_PIVOT_TO_WRIST_PIVOT**2 + WRIST_PIVOT_TO_NUT**2 - (2 * MOTOR_PIVOT_TO_WRIST_PIVOT * WRIST_PIVOT_TO_NUT * math.cos(toRads(BETA))))
MOTOR_POS_ROTS = cmToRots(LEAD_SCREW_LENGTH * 2.54) # <- turn inches to cm
print(LEAD_SCREW_LENGTH )
print(MOTOR_POS_ROTS)
from typing import Tuple


def get_direction(ball_angle: float) -> int:
    if ball_angle >= 345 or ball_angle <= 15:
        return 0
    return -1 if ball_angle < 180 else 1


def getDirectionBy4(angle: float) -> int:
    if angle >= 345 or angle < 15:
        return 0
    elif angle >= 15 and angle < 165:
        return 1
    elif angle >= 165 and angle < 195:
        return 2
    elif angle >= 195 and angle < 345:
        return 3


def getDistance(position1: Tuple[float, float], position2: Tuple[float, float]) -> float:
    return ((position1[0]-position2[0])**2+(position1[1]-position2[1])**2)**0.5


def correctAngle(angle: float) -> float:
    if angle < 0:
        angle = angle % -360+360
    else:
        angle%=360
    return angle
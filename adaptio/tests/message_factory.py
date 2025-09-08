def GetAdaptioVersion():
    return {"name": "GetAdaptioVersion", "payload": {}}


def ServiceModeTracking():
    return {"name": "ServiceModeTracking", "payload": {}}


def StartTracking():
    return {
        "name": "StartTracking",
        "payload": {
            "horizontal_offset": 0,
            "vertical_offset": 0,
            "joint_tracking_mode": 0,
        },
    }


def LaserTorchCalSet():
    return {
        "name": "LaserTorchCalSet",
        "payload": {"distanceLaserTorch": 0, "stickout": 0, "scannerMountAngle": 0},
    }


def LaserTorchCalGet():
    return {"name": "LaserTorchCalGet", "payload": {}}


def WeldObjectCalStart():
    return {
        "name": "WeldObjectCalStart",
        "payload": {"wireDiameter": 0, "stickout": 0, "weldObjectRadius": 0},
    }


def WeldObjectCalStop():
    return {"name": "WeldObjectCalStop", "payload": {}}


def WeldObjectCalLeftPos():
    return {"name": "WeldObjectCalLeftPos", "payload": {}}


def WeldObjectCalRightPos():
    return {"name": "WeldObjectCalRightPos", "payload": {}}


def WeldObjectCalGet():
    return {"name": "WeldObjectCalGet", "payload": {}}


def WeldObjectCalSet():
    return {"name": "WeldObjectCalSet", "payload": {}}


def GetSlidesPosition():
    return {"name": "GetSlidesPosition", "payload": {}}


def ServiceModeStop():
    return {"name": "ServiceModeStop", "payload": {}}


def ServiceModeKinematicsControl():
    return {"name": "ServiceModeKinematicsControl", "payload": {}}


def SetSlidesPosition():
    return {"name": "SetSlidesPosition", "payload": {"horizontal": 0, "vertical": 0}}


def SetJointGeometry():
    return {
        "name": "SetJointGeometry",
        "payload": {
            "upper_joint_width_mm": 58.0,
            "groove_depth_mm": 37.0,
            "left_joint_angle_rad": 0.524,
            "right_joint_angle_rad": 0.524,
            "left_max_surface_angle_rad": 0.26,
            "right_max_surface_angle_rad": 0.26,
        },
    }


def SetWeldAxisData():
    return {"name": "SetWeldAxisData", "payload": {"velocity": 0}}

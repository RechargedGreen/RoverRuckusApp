package org.firstinspires.ftc.teamcode.botA.hardware

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.david.rechargedkotlinlibrary.internal.hardware.devices.OptimumDcMotorEx
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.ConfigData
import com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain.DiffDrive
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.qualcomm.robotcore.hardware.DcMotorSimple

class BotADrive(robot:RobotTemplate):DiffDrive(
        robot = robot,
        leftMotors = arrayOf(
                OptimumDcMotorEx(ConfigData(robot, 0,"lf")),
                OptimumDcMotorEx(ConfigData(robot, 0,"lb"))
        ),
        rightMotors = arrayOf(
                OptimumDcMotorEx(ConfigData(robot, 0,"rf"), direction =  DcMotorSimple.Direction.REVERSE),
                OptimumDcMotorEx(ConfigData(robot, 0,"rb"), direction =  DcMotorSimple.Direction.REVERSE)
        ),
        RADIUS = 2.0,
        WHEEL_GEAR_RATIO = 42.0 / 40.0,
        TRACK_WIDTH = 0.0,
        CROSSTRACK_PID_COEFFICIENTS = PIDCoefficients(0.0, 0.0, 0.0),
        DISPLACEMENT_PID_COEFFICIENTS = PIDCoefficients(0.0, 0.0, 0.0),
        kV = 0.0,
        MAX_ACCEL = 0.0,
        MAX_TURN_ACCEL = 0.0,
        MAX_VEL = 0.0
        )
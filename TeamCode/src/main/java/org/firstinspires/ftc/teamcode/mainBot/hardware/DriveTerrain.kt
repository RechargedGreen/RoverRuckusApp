package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.david.rechargedkotlinlibrary.internal.hardware.PIDController
import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedDcMotorEx
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.ConfigData
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.imu.SimplifiedBNO055
import com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain.DiffDrive
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
private val autoRunMode = DcMotor.RunMode.RUN_USING_ENCODER
private val teleRunMode = DcMotor.RunMode.RUN_USING_ENCODER
class DriveTerrain(robot: RobotTemplate) : DiffDrive(
        robot = robot,
        leftMotors = arrayOf(
                CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "lf", mode = if (robot.opMode.isAutonomous()) autoRunMode else teleRunMode), robot.getHub(0)),
                CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "lb", mode = if (robot.opMode.isAutonomous()) autoRunMode else teleRunMode), robot.getHub(0))
        ),
        rightMotors = arrayOf(
                CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "rf", direction = DcMotorSimple.Direction.REVERSE, mode = if (robot.opMode.isAutonomous()) autoRunMode else teleRunMode), robot.getHub(0)),
                CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "rb", direction = DcMotorSimple.Direction.REVERSE, mode = if (robot.opMode.isAutonomous()) autoRunMode else teleRunMode), robot.getHub(0))
        ),
        RADIUS = 2.0,
        WHEEL_GEAR_RATIO = 42.0 / 40.0,
        TRACK_WIDTH = 0.0,
        CROSSTRACK_PID_COEFFICIENTS = PIDCoefficients(0.0, 0.0, 0.0),
        DISPLACEMENT_PID_COEFFICIENTS = PIDCoefficients(0.0, 0.0, 0.0),
        kV = 0.0,
        MAX_ACCEL = 0.0,
        MAX_TURN_ACCEL = 0.0,
        MAX_VEL = 0.0,
        imu = SimplifiedBNO055(HardwareMaker.BNO055IMU.make(robot.hMap, "imu", true, BNO055IMU.SensorMode.GYRONLY))
) {
    enum class AngleFollowSpeeds(val controller:PIDController, val speed:Double) {
        FAST(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients(0.0, 0.0, 0.0)), 1.0),
    }

    fun startFollowingAngle_setConstants(angleFollowSpeed: AngleFollowSpeeds = AngleFollowSpeeds.FAST, angle:Double) {
        startFollowingAngle(angleFollowSpeed.controller, angleFollowSpeed.speed, angle)
    }
}
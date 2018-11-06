package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.drive.Drive
import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.david.rechargedkotlinlibrary.internal.hardware.PIDController
import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedDcMotorEx
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.imu.SimplifiedBNO055
import com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain.DiffDrive
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.mpTuningBot.DriveConstants

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
        RADIUS = DriveConstants.WHEEL_RADIUS,
        WHEEL_GEAR_RATIO = DriveConstants.GEAR_RATIO,
        TRACK_WIDTH = DriveConstants.TRACK_WIDTH,
        CROSSTRACK_PID = PIDCoefficients(0.0, 0.0, 0.0),
        DISPLACEMENT_PID = PIDCoefficients(0.0, 0.0, 0.0),
        kV = DriveConstants.kV,
        MAX_ACCEL = DriveConstants.BASE_CONSTRAINTS.maximumAcceleration,
        MAX_TURN_ACCEL = DriveConstants.BASE_CONSTRAINTS.maximumAngularAcceleration,
        MAX_VEL = DriveConstants.BASE_CONSTRAINTS.maximumVelocity,
        imu = SimplifiedBNO055(HardwareMaker.BNO055IMU.make(robot.hMap, "imu", true, BNO055IMU.SensorMode.IMU)),
        encoderTicksToInches = { ticks -> DriveConstants.encoderTicksToInches(ticks) }
) {
    enum class AngleFollowSpeeds(val controller: PIDController, val speed: Double) {
        FAST(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients(0.0, 0.0, 0.0)), 1.0),
    }

    fun startFollowingAngle_setConstants(angleFollowSpeed: AngleFollowSpeeds = AngleFollowSpeeds.FAST, angle: Double) {
        startFollowingAngle(angleFollowSpeed.controller, angleFollowSpeed.speed, angle)
    }

    enum class WallFollows() {
        OWN_CRATER_TO_DEPOT,
        OPPOSING_CRATER_TO_DEPOT,
        DEPOT_TO_OPPOSING_CRATER,
        DEPOT_TO_OWN_CRATER
    }

    fun followWall(type: WallFollows) {

    }
}
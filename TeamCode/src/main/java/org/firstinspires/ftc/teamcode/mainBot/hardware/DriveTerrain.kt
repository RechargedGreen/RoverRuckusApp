package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.david.rechargedkotlinlibrary.internal.hardware.PIDController
import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedDcMotorEx
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.imu.SimplifiedBNO055
import com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain.DiffDrive
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.mpTuningBot.DriveConstants
import kotlin.math.absoluteValue

class DriveTerrain(val robot: RobotTemplate) : DiffDrive(
        robot = robot,
        leftMotors = arrayOf(
                CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "lf"), robot.getHub(0)),
                CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "lb"), robot.getHub(0))
        ),
        rightMotors = arrayOf(
                CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "rf", direction = DcMotorSimple.Direction.REVERSE), robot.getHub(0)),
                CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "rb", direction = DcMotorSimple.Direction.REVERSE), robot.getHub(0))
        ),
        CROSSTRACK_PID = PIDCoefficients(0.0, 0.0, 0.0),
        DISPLACEMENT_PID = PIDCoefficients(0.0, 0.0, 0.0),
        imu = SimplifiedBNO055(HardwareMaker.BNO055IMU.make(robot.hMap, "imu", true, BNO055IMU.SensorMode.IMU)),
        encoderTicksToInches = { ticks -> DriveConstants.encoderTicksToInches(ticks) },
        baseConstraints = DriveConstants.BASE_CONSTRAINTS
) {
    enum class AngleFollowSpeeds(val controller: PIDController, val speed: Double) {
        PARK(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients(0.005, 0.0, 0.002)), 0.15),
        FAST(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients(0.005, 0.0, 0.002)), 1.0),
        SLOW(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients(0.005, 0.0, 0.002)), 0.3),
        // 0.005, 0.0, 0.001 at lm1
        TURN(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients(0.005, 0.0, 0.002)), 0.0),
        STRAFE(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients(0.005, 0.0, 0.002)), 0.0)
    }

    fun startFollowingAngle_setConstants(angleFollowSpeed: AngleFollowSpeeds = AngleFollowSpeeds.FAST, angle: Double, reverse:Boolean = false, type:AnglePIDType, maxTurnPower:Double = 1.0) {
        startFollowingAngle(angleFollowSpeed.controller, if(reverse) -angleFollowSpeed.speed else angleFollowSpeed.speed, angle, type, maxTurnPower)
    }

    fun pidTurn(target:Double, threshold:Double = 2.0, stop:Boolean = true, maxTurnPower: Double = 1.0) {
        startFollowingAngle_setConstants(AngleFollowSpeeds.TURN, target, false, AnglePIDType.POINT_TURN, maxTurnPower)
        robot.opMode.waitTill { (imu.getZ(AngleUnit.DEGREES) - target).absoluteValue < threshold }
        if(stop)
            stop()
    }

    fun deadReckonPID(ticks:Int, angle:Double, speed:AngleFollowSpeeds = AngleFollowSpeeds.FAST, stop:Boolean = true){
        if(ticks != 0) {
            val reverse = ticks < 0
            resetEncoders()
            startFollowingAngle_setConstants(speed, angle, reverse, AnglePIDType.STRAIGHT)
            if (reverse)
                robot.opMode.waitTill { (leftTicks() + rightTicks()) < ticks }
            else
                robot.opMode.waitTill { (leftTicks() + rightTicks()) > ticks }
        }
        if(stop)
            stop()
    }

    fun strafeAroundLeft(target:Double, threshold:Double = 2.0, stop:Boolean = true, maxTurnPower: Double = 1.0){
        startFollowingAngle_setConstants(AngleFollowSpeeds.STRAFE, target, false, AnglePIDType.TURN_AROUND_LEFT, maxTurnPower)
        robot.opMode.waitTill { (imu.getZ(AngleUnit.DEGREES) - target).absoluteValue < threshold }
        if(stop)
            stop()
    }

    fun strafeAroundRight(target:Double, threshold:Double = 2.0, stop:Boolean = true, maxTurnPower: Double = 1.0){
        startFollowingAngle_setConstants(AngleFollowSpeeds.STRAFE, target, false, AnglePIDType.TURN_AROUND_RIGHT, maxTurnPower)
        robot.opMode.waitTill { (imu.getZ(AngleUnit.DEGREES) - target).absoluteValue < threshold }
        if(stop)
            stop()
    }

    fun runTime(power:Double, seconds:Double, stop:Boolean = true){
        if(seconds != 0.0) {
            openLoopArcade(power)
            robot.opMode.sleepSeconds(seconds)
        }
        if(stop)
            stop()
    }
}
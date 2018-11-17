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
        TRACK_WIDTH = DriveConstants.TRACK_WIDTH,
        CROSSTRACK_PID = PIDCoefficients(0.0, 0.0, 0.0),
        DISPLACEMENT_PID = PIDCoefficients(0.0, 0.0, 0.0),
        kV = DriveConstants.kV,
        kA = DriveConstants.kA,
        kStatic = DriveConstants.kStatic,
        imu = SimplifiedBNO055(HardwareMaker.BNO055IMU.make(robot.hMap, "imu", true, BNO055IMU.SensorMode.IMU)),
        encoderTicksToInches = { ticks -> DriveConstants.encoderTicksToInches(ticks) },
        baseConstraints = DriveConstants.BASE_CONSTRAINTS
) {
    enum class AngleFollowSpeeds(val controller: PIDController, val speed: Double) {
        FAST(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients(0.01, 0.0, 0.0013)), 1.0),
        SLOW(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients(0.01, 0.0, 0.0013)), 0.3),
        TURN(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients(0.005, 0.0, 0.001)), 0.0)
    }

    fun startFollowingAngle_setConstants(angleFollowSpeed: AngleFollowSpeeds = AngleFollowSpeeds.FAST, angle: Double, reverse:Boolean = false) {
        startFollowingAngle(angleFollowSpeed.controller, if(reverse) -angleFollowSpeed.speed else angleFollowSpeed.speed, angle)
    }

    enum class WallFollows() {
        OWN_CRATER_TO_DEPOT,
        OPPOSING_CRATER_TO_DEPOT,
        DEPOT_TO_OPPOSING_CRATER,
        DEPOT_TO_OWN_CRATER
    }

    fun followWall(type: WallFollows) {

    }

    fun pidTurn(target:Double, threshold:Double = 2.0){
        startFollowingAngle_setConstants(AngleFollowSpeeds.TURN, target)
        robot.opMode.waitTill { (imu.getZ(AngleUnit.DEGREES) - target).absoluteValue < threshold }
        stop()
    }

    fun deadReckonPID(ticks:Int, angle:Double, speed:AngleFollowSpeeds = AngleFollowSpeeds.FAST){
        val reverse = ticks < 0
        resetEncoders()
        startFollowingAngle_setConstants(speed, angle, reverse)
        if(reverse)
            robot.opMode.waitTill { (leftTicks() + rightTicks()) < ticks}
        else
            robot.opMode.waitTill { (leftTicks() + rightTicks()) > ticks}
        stop()
    }

    fun runTime(power:Double, seconds:Double){
        openLoopArcade(power)
        robot.opMode.sleepSeconds(seconds)
        stop()
    }
}
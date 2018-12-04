package com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.drive.Drive
import com.acmerobotics.roadrunner.drive.TankDrive
import com.acmerobotics.roadrunner.drive.TankKinematics
import com.acmerobotics.roadrunner.followers.RamseteFollower
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints
import com.acmerobotics.roadrunner.util.Angle
import com.david.rechargedkotlinlibrary.internal.hardware.PIDController
import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedDcMotorEx
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.imu.SimplifiedBNO055
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.david.rechargedkotlinlibrary.internal.util.MathUtil
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import java.util.*
import kotlin.math.absoluteValue

/**
 * Created by David Lukens on 8/7/2018.
 */
abstract class DiffDrive(
        private val robot: RobotTemplate,
        private val leftMotors: Array<CachedDcMotorEx>,
        private val rightMotors: Array<CachedDcMotorEx>,
        mode: DcMotor.RunMode = DcMotor.RunMode.RUN_USING_ENCODER,
        zeroPowerBehavior: DcMotor.ZeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE,
        val imu: SimplifiedBNO055,
        val DISPLACEMENT_PID: PIDCoefficients,
        val CROSSTRACK_PID: PIDCoefficients,
        val encoderTicksToInches: (Int) -> Double,
        baseConstraints: DriveConstraints
) : MTSubsystem {
    init {
        leftMotors.forEach {
            it.mode = mode
            it.zeroPowerBehavior = zeroPowerBehavior
        }
        rightMotors.forEach {
            it.mode = mode
            it.zeroPowerBehavior = zeroPowerBehavior
        }
        robot.thread.addSubsystem(this)
    }

    enum class ControlLoopStates {
        OPEN_LOOP,
        DRIVING_AT_ANGLE
    }

    private var controlState = ControlLoopStates.OPEN_LOOP

    override fun update() {
        imu.clearCaches()
        imu.checkAngleCache()
        if(controlState != ControlLoopStates.DRIVING_AT_ANGLE)
            lastAngleFollowerError = null
        when (controlState) {
            ControlLoopStates.OPEN_LOOP -> {
                val powers = openLoopWheelPowers.copy()
                setMotorPowers(powers.l, powers.r)
            }
            ControlLoopStates.DRIVING_AT_ANGLE -> {
                val err = MathUtil.norm(imu.getZ() - followAngleData.angle, AngleUnit.DEGREES)
                val data = followAngleData
                lastAngleFollowerError = err
                val turn = Range.clip(-followAngleData.controller.update(err), -data.maxTurnPower, data.maxTurnPower)
                when(followAngleData.type){
                    AnglePIDType.POINT_TURN -> internalArcade(0.0, turn)
                    AnglePIDType.STRAIGHT -> internalArcade(data.power, turn)
                    AnglePIDType.TURN_AROUND_LEFT -> internalArcade(turn, turn)//todo remove clip
                    AnglePIDType.TURN_AROUND_RIGHT -> internalArcade(-turn, turn)
                }
            }
        }
    }

    private fun internalArcade(x:Double, heading:Double){
        val left = x - heading
        val right = x + heading
        val max = Collections.max(listOf(left.absoluteValue, right.absoluteValue, 1.0))
        setMotorPowers(left / max, right / max)
    }

    fun setMotorPowers(left: Double, right: Double) {
        leftMotors.forEach { it.power = left }
        rightMotors.forEach { it.power = right }
    }

    fun leftRawTicks(): Int {
        var sum = 0
        leftMotors.forEach { sum += it.encoder.getRawTicks() }
        return sum / leftMotors.size
    }

    fun rightRawTicks(): Int {
        var sum = 0
        rightMotors.forEach { sum += it.encoder.getRawTicks() }
        return sum / rightMotors.size
    }

    fun leftTicks(): Int {
        var sum = 0
        leftMotors.forEach { sum += it.encoder.getTicks() }
        return sum / leftMotors.size
    }

    fun rightTicks(): Int {
        var sum = 0
        rightMotors.forEach { sum += it.encoder.getTicks() }
        return sum / rightMotors.size
    }

    fun resetRightEncoders() = rightMotors.forEach { it.encoder.reset() }

    fun resetLeftEncoders() = leftMotors.forEach { it.encoder.reset() }

    fun resetEncoders() {
        resetLeftEncoders()
        resetRightEncoders()
    }

    override fun start() {
    }

    fun stop() = openLoopPowerWheels(0.0, 0.0)


    private var maxTurnPower = 1.0

    enum class AnglePIDType{
        POINT_TURN,
        TURN_AROUND_LEFT,
        TURN_AROUND_RIGHT,
        STRAIGHT
    }

    fun openLoopPowerWheels(l: Double, r: Double) {
        controlState = ControlLoopStates.OPEN_LOOP
        openLoopWheelPowers = SidePowers(l = l, r = r)
    }

    private var openLoopWheelPowers = SidePowers(l = 0.0, r = 0.0)

    private data class SidePowers(val l: Double, val r: Double)

    fun openLoopArcade(x:Double = 0.0, heading:Double = 0.0){
        val l = x - heading
        val r = x + heading
        val max = Collections.max(listOf(l.absoluteValue, r.absoluteValue, 1.0))
        openLoopPowerWheels(l = l / max, r = r / max)
    }

    var lastAngleFollowerError:Double? = null
    private var followAngleData = FollowAngleData(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients()), 0.0, 0.0, AnglePIDType.STRAIGHT, 1.0)

    data class FollowAngleData(val controller: PIDController, val power: Double, val angle: Double, val type:AnglePIDType, val maxTurnPower:Double)

    fun startFollowingAngle(controller: PIDController, power: Double = 0.0, angle: Double, type:AnglePIDType, maxTurnPower: Double = 1.0) {
        followAngleData = FollowAngleData(controller = controller, power = power, angle = angle, type = type, maxTurnPower = maxTurnPower)
        controlState = ControlLoopStates.DRIVING_AT_ANGLE
    }
}
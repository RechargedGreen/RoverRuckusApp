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
import com.david.rechargedkotlinlibrary.internal.hardware.PIDController
import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedDcMotorEx
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.imu.SimplifiedBNO055
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.david.rechargedkotlinlibrary.internal.util.MathUtil
import com.qualcomm.robotcore.hardware.DcMotor
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
        val WHEEL_GEAR_RATIO: Double = 1.0,
        val RADIUS: Double = 2.0,
        kA: Double = 0.0,
        kV: Double,
        kStatic: Double = 0.0,
        MAX_VEL: Double = 1.0 / kV,
        MAX_ACCEL: Double,
        MAX_TURN_ACCEL: Double,
        TRACK_WIDTH: Double = 1.0,
        val imu: SimplifiedBNO055,
        val DISPLACEMENT_PID: PIDCoefficients,
        val CROSSTRACK_PID: PIDCoefficients
) : TankDrive(TRACK_WIDTH), MTSubsystem, TunableDrive {
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
        FOLLOW_TRAJECTORY,
        OPEN_LOOP,
        DRIVING_AT_ANGLE
    }

    private val HARD_MAX_VEL: Double = 1.0 / kV
    val MOTOR_TYPE = leftMotors[0].motorType
    val ENCODER_SCALER = 1.0 / WHEEL_GEAR_RATIO
    val hardConstraints = TankConstraints(DriveConstraints(1.0 / kV, MAX_VEL, MAX_ACCEL, MAX_TURN_ACCEL), trackWidth)
    val follower = TankPIDVAFollower(this, displacementCoeffs = DISPLACEMENT_PID, crossTrackCoeffs = CROSSTRACK_PID, kV = kV, kStatic = kStatic, kA = kA)
    private var controlState = ControlLoopStates.OPEN_LOOP
    override fun getHeading(): Double? = imu.getZ(AngleUnit.RADIANS) // needs to be radians for roadrunner

    override fun getDrive(): Drive = this

    override fun update() {
        imu.clearCaches()
        imu.checkAngleCache()
        updatePoseEstimate()
        when (controlState) {
            ControlLoopStates.FOLLOW_TRAJECTORY -> follower.update(poseEstimate)
            ControlLoopStates.OPEN_LOOP         -> {
                val powers = openLoopWheelPowers.copy()
                setMotorPowers(powers.l, powers.r)
            }
            ControlLoopStates.DRIVING_AT_ANGLE  -> {
                val err = MathUtil.norm(followAngleData.angle - imu.getZ(), AngleUnit.DEGREES)
                lastAngleFollowerError = err
                val turn = followAngleData.controller.update(err)
                val left = followAngleData.power + turn
                val right = followAngleData.power - turn
                val max = Collections.max(listOf(left.absoluteValue, right.absoluteValue, 1.0))
                setMotorPowers(left / max, right / max)
            }
        }
    }

    override fun setVel(vel: Pose2d) {
        val powers = TankKinematics.robotToWheelAccelerations(vel, trackWidth)
        openLoopPowerWheels(powers[0], powers[1])
    }
    override fun getMaxWheelMotorRPM() = MOTOR_TYPE.maxRPM
    override fun getWheelRadius() = RADIUS
    override fun getWheelGearRatio() = WHEEL_GEAR_RATIO
    override fun getGyro() = imu

    override fun setMotorPowers(left: Double, right: Double) {
        leftMotors.forEach { it.power = left }
        rightMotors.forEach { it.power = right }
    }

    fun leftRawTicks(): Int {
        var sum = 0
        leftMotors.forEach { sum += it.encoder.getRawTicks() }
        return sum / 2
    }

    fun rightRawTicks(): Int {
        var sum = 0
        rightMotors.forEach { sum += it.encoder.getRawTicks() }
        return sum / 2
    }

    fun leftRawRadians(): Double {
        var sum = 0.0
        leftMotors.forEach { sum += it.encoder.getRawRadians() }
        return sum / 2
    }

    fun rightRawRadians(): Double {
        var sum = 0.0
        leftMotors.forEach { sum += it.encoder.getRawRadians() }
        return sum / 2
    }

    fun leftTicks(): Int {
        var sum = 0
        leftMotors.forEach { sum += it.encoder.getTicks() }
        return sum / 2
    }

    fun rightTicks(): Int {
        var sum = 0
        rightMotors.forEach { sum += it.encoder.getTicks() }
        return sum / 2
    }

    fun leftRadians(): Double {
        var sum = 0.0
        leftMotors.forEach { sum += it.encoder.getRadians() }
        return sum / 2
    }

    fun rightRadians(): Double {
        var sum = 0.0
        leftMotors.forEach { sum += it.encoder.getRadians() }
        return sum / 2
    }


    fun resetRightEncoders() {
        rightMotors.forEach { it.encoder.reset() }
    }

    fun resetLeftEncoders() {
        leftMotors.forEach { it.encoder.reset() }
    }

    fun resetEncoders() {
        resetLeftEncoders()
        resetRightEncoders()
    }

    override fun getWheelPositions(): List<Double> {
        val positions = LinkedList<Double>()
        positions.add(radiansToInches(leftRawRadians()))
        positions.add(radiansToInches(rightRawRadians()))
        return positions
    }

    fun radiansToInches(radians: Double) = radians * RADIUS * ENCODER_SCALER

    override fun start() {
    }

    fun stop() = openLoopPowerWheels(0.0, 0.0)

    fun startDrivingAtAngle(controller: PIDController = PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients()), power: Double = 1.0, angle: Double) {
        followAngleData = FollowAngleData(controller = controller, power = power, angle = MathUtil.norm(angle, AngleUnit.DEGREES))
        controlState = ControlLoopStates.DRIVING_AT_ANGLE
    }

    fun startFollowingTrajectory(trajectory: Trajectory) {
        follower.followTrajectory(trajectory)
        controlState = ControlLoopStates.FOLLOW_TRAJECTORY
    }

    fun followingTrajectory(): Boolean = follower.isFollowing()

    fun waitOnFollower(condition: () -> Boolean = { true }, action: Runnable? = null) {
        while (robot.opMode.opModeIsActive() && followingTrajectory() && condition()) action?.run()
        stop()
    }

    fun waitOnTrajectory(condition: () -> Boolean = { true }, action: Runnable? = null, trajectory: Trajectory) {
        startFollowingTrajectory(trajectory)
        waitOnFollower(condition, action)
    }

    fun trajectoryBuilder(pos: Pose2d = poseEstimate, constraints: TankConstraints = hardConstraints) = TrajectoryBuilder(pos, constraints)

    fun openLoopPowerWheels(l: Double, r: Double) {
        controlState = ControlLoopStates.OPEN_LOOP
        openLoopWheelPowers = SidePowers(l = l, r = r)
    }
    var openLoopWheelPowers = SidePowers(l = 0.0, r = 0.0)
    data class SidePowers(val l: Double, val r: Double)

    var lastAngleFollowerError = 0.0
    private var followAngleData = FollowAngleData(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients()), 0.0, 0.0)
    data class FollowAngleData(val controller: PIDController, val power: Double, val angle: Double)
    fun startFollowingAngle(controller: PIDController, power: Double = 0.0, angle: Double) {
        followAngleData = FollowAngleData(controller = controller, power = power, angle = angle)
        controlState = ControlLoopStates.DRIVING_AT_ANGLE
    }
}
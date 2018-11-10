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
        kA: Double,
        kV: Double,
        kStatic: Double,
        TRACK_WIDTH: Double,
        val imu: SimplifiedBNO055,
        val DISPLACEMENT_PID: PIDCoefficients,
        val CROSSTRACK_PID: PIDCoefficients,
        val encoderTicksToInches: (Int) -> Double,
        baseConstraints: DriveConstraints
) : TankDrive(TRACK_WIDTH), MTSubsystem {
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

    val mainConstraints = TankConstraints(baseConstraints, trackWidth)
    val follower = TankPIDVAFollower(this, displacementCoeffs = DISPLACEMENT_PID, crossTrackCoeffs = CROSSTRACK_PID, kV = kV, kStatic = kStatic, kA = kA)
    private var controlState = ControlLoopStates.OPEN_LOOP

    override fun getExternalHeading() = imu.getZ(AngleUnit.RADIANS) // radians for road runner

    override fun update() {
        imu.clearCaches()
        imu.checkAngleCache()
        updatePoseEstimate()
        when (controlState) {
            ControlLoopStates.FOLLOW_TRAJECTORY -> follower.update(poseEstimate)
            ControlLoopStates.OPEN_LOOP -> {
                val powers = openLoopWheelPowers.copy()
                setMotorPowers(powers.l, powers.r)
            }
            ControlLoopStates.DRIVING_AT_ANGLE -> {
                val err = MathUtil.norm(imu.getZ() - followAngleData.angle, AngleUnit.DEGREES)
                lastAngleFollowerError = err
                val turn = followAngleData.controller.update(err)
                val left = followAngleData.power + turn
                val right = followAngleData.power - turn
                val max = Collections.max(listOf(left.absoluteValue, right.absoluteValue, 1.0))
                setMotorPowers(left / max, right / max)
            }
        }
    }

    override fun setMotorPowers(left: Double, right: Double) {
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

    fun leftRawInches() = encoderTicksToInches(leftRawTicks())
    fun rightRawInches() = encoderTicksToInches(rightRawTicks())

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

    override fun getWheelPositions(): List<Double> = listOf(leftRawInches(), rightRawInches())

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

    fun trajectoryBuilder(pos: Pose2d = poseEstimate, constraints: TankConstraints = mainConstraints) = TrajectoryBuilder(pos, constraints)

    fun openLoopPowerWheels(l: Double, r: Double) {
        controlState = ControlLoopStates.OPEN_LOOP
        openLoopWheelPowers = SidePowers(l = l, r = r)
    }

    private var openLoopWheelPowers = SidePowers(l = 0.0, r = 0.0)

    private data class SidePowers(val l: Double, val r: Double)

    fun openLoopSetVelocity(vel:Pose2d) {
        val powers = TankKinematics.robotToWheelVelocities(vel, trackWidth)
        openLoopPowerWheels(l = powers[0], r = powers[1])
    }

    fun openLoopArcade(x:Double = 0.0, heading:Double = 0.0){
        val l = x - heading
        val r = x + heading
        val max = Collections.max(listOf(l.absoluteValue, r.absoluteValue, 1.0))
        openLoopPowerWheels(l = l / max, r = r / max)
    }

    var lastAngleFollowerError = 0.0
    private var followAngleData = FollowAngleData(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients()), 0.0, 0.0)

    data class FollowAngleData(val controller: PIDController, val power: Double, val angle: Double)

    fun startFollowingAngle(controller: PIDController, power: Double = 0.0, angle: Double) {
        followAngleData = FollowAngleData(controller = controller, power = power, angle = angle)
        controlState = ControlLoopStates.DRIVING_AT_ANGLE
    }
}
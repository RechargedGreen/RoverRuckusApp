package com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.drive.TankDrive
import com.acmerobotics.roadrunner.followers.RamseteFollower
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints
import com.david.rechargedkotlinlibrary.internal.hardware.devices.OptimumDcMotorEx
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.odometry.Localizer
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.david.rechargedkotlinlibrary.internal.hardware.states.ControlLoopStates
import com.david.rechargedkotlinlibrary.internal.roadRunner.RamseteConstraints
import com.qualcomm.robotcore.hardware.DcMotor
import java.util.*

/**
 * Created by David Lukens on 8/7/2018.
 */
abstract class DiffDrive(
        private val robot: RobotTemplate,
        private val leftMotors: Array<OptimumDcMotorEx>,
        private val rightMotors: Array<OptimumDcMotorEx>,
        mode: DcMotor.RunMode = DcMotor.RunMode.RUN_USING_ENCODER,
        zeroPowerBehavior: DcMotor.ZeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE,
        val WHEEL_GEAR_RATIO: Double = 1.0,
        val RADIUS: Double = 2.0,
        DISPLACEMENT_PID_COEFFICIENTS: PIDCoefficients,
        CROSSTRACK_PID_COEFFICIENTS: PIDCoefficients,
        ramseteConstraints: RamseteConstraints = RamseteConstraints(),
        kA: Double = 0.0,
        kV: Double,
        kStatic: Double = 0.0,
        MAX_VEL: Double = 1.0 / kV,
        MAX_ACCEL: Double,
        MAX_TURN_ACCEL: Double,
        var followerType: Follower = Follower.PIDVA,
        TRACK_WIDTH: Double,
        localizerArg: Localizer? = null)
    : TankDrive(TRACK_WIDTH), MTSubsystem, Localizer {
    private val HARD_MAX_VEL: Double = 1.0 / kV

    val MOTOR_TYPE = leftMotors[0].motorType
    val ENCODER_SCALER = 1.0 / WHEEL_GEAR_RATIO

    override var biasPose = Pose2d(Vector2d(0.0, 0.0), 0.0)
    private var controlState = ControlLoopStates.OPEN
    private val localizerArg = localizerArg ?: this

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

    private val baseConstraints = DriveConstraints(1.0 / kV, MAX_VEL, MAX_ACCEL, MAX_TURN_ACCEL)
    val hardConstraints = TankConstraints(baseConstraints, trackWidth)

    fun waitOnFollower(condition: () -> Boolean = { true }, action: Runnable? = null) {
        while (robot.opMode.opModeIsActive() && followingTrajectory() && condition()) action?.run()
    }

    fun waitOnTrajectory(condition: () -> Boolean = { true }, action: Runnable? = null, trajectory: Trajectory, followType: Follower = followerType) {
        startFollowingTrajectory(trajectory, followType)
        waitOnFollower(condition, action)
    }

    fun trajectoryBuilder(pos: Pose2d = localizerArg.getPos(), constraints: TankConstraints = hardConstraints) = TrajectoryBuilder(pos, constraints)

    private val followerPIDVA = TankPIDVAFollower(drive = this, displacementCoeffs = DISPLACEMENT_PID_COEFFICIENTS, crossTrackCoeffs = CROSSTRACK_PID_COEFFICIENTS, kV = kV, kA = kA, kStatic = kStatic)
    private val followerRamsete = RamseteFollower(drive = this, b = ramseteConstraints.b, zeta = ramseteConstraints.zeta, kV = kV, kA = kA, kStatic = kStatic)

    enum class Follower {
        RAMSETE,
        PIDVA,
    }

    fun getFollower(type: Follower = Follower.PIDVA): TrajectoryFollower {
        return when (type) {
            Follower.PIDVA   -> followerPIDVA
            Follower.RAMSETE -> followerRamsete
        }
    }

    override fun getWheelPositions(): List<Double> {
        var lSum = 0.0
        leftMotors.forEach { lSum += it.getRawRadians() }
        var rSum = 0.0
        rightMotors.forEach { rSum += it.getRawRadians() }

        val positions = LinkedList<Double>()
        positions.add(radiansToInches(lSum / leftMotors.size))
        positions.add(radiansToInches(rSum / rightMotors.size))
        return positions
    }

    fun radiansToInches(radians: Double) = radians * RADIUS * ENCODER_SCALER

    override fun setMotorPowers(left: Double, right: Double) {
        leftMotors.forEach { it.power = left }
        rightMotors.forEach { it.power = right }
    }

    override fun getRawPos() = poseEstimate
    override fun updatePos() = updatePoseEstimate()

    private var activeTrajectoryFollower: TrajectoryFollower? = null

    override fun update() {
        localizerArg.updatePos()
        when (controlState) {
            ControlLoopStates.CLOSED -> {
                val follower = activeTrajectoryFollower
                follower?.update(localizerArg.getPos())
            }
            ControlLoopStates.OPEN   -> {
                val powers = openLoopWheelPowers.copy()
                setMotorPowers(powers.l, powers.r)
            }
        }
    }

    override fun start() {
    }

    fun startFollowingTrajectory(trajectory: Trajectory, followType: Follower = followerType) {
        val follower = getFollower(followType)
        follower.followTrajectory(trajectory)
        setActiveTrajectoryFollower(follower)
    }

    fun followingTrajectory(): Boolean = activeTrajectoryFollower?.isFollowing() ?: false

    fun setActiveTrajectoryFollower(follower: TrajectoryFollower) {
        controlState = ControlLoopStates.CLOSED
        activeTrajectoryFollower = follower
    }

    fun openLoopPowerWheels(l: Double, r: Double) {
        controlState = ControlLoopStates.OPEN
        openLoopWheelPowers = SidePowers(l = l, r = r)
    }

    var openLoopWheelPowers = SidePowers(l = 0.0, r = 0.0)

    data class SidePowers(val l: Double, val r: Double)
}
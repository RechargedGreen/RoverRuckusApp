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
import com.david.rechargedkotlinlibrary.internal.hardware.PIDController
import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedDcMotorEx
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.imu.SimplifiedBNO055
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.odometry.Localizer
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.david.rechargedkotlinlibrary.internal.roadRunner.RamseteConstraints
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
        localizerArg: Localizer? = null,
        val imu:SimplifiedBNO055)
    : TankDrive(TRACK_WIDTH), MTSubsystem, Localizer {
    enum class ControlLoopStates {
        FOLLOW_TRAJECTORY,
        OPEN_LOOP,
        DRIVING_AT_ANGLE
    }

    private var followAngleData = FollowAngleData(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients()), 0.0, 0.0)
    data class FollowAngleData(val controller:PIDController, val power:Double, val angle:Double)

    fun startFollowingAngle(controller: PIDController, power:Double = 0.0, angle:Double){
        followAngleData = FollowAngleData(controller = controller, power = power, angle = angle)
        controlState = ControlLoopStates.DRIVING_AT_ANGLE
    }

    private val HARD_MAX_VEL: Double = 1.0 / kV

    var lastAngleFollowerError = 0.0

    val MOTOR_TYPE = leftMotors[0].motorType
    val ENCODER_SCALER = 1.0 / WHEEL_GEAR_RATIO

    override var biasPose = Pose2d(Vector2d(0.0, 0.0), 0.0)
    private var controlState = ControlLoopStates.OPEN_LOOP
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
        stop()
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

    fun leftRawTicks():Int{
        var sum = 0
        leftMotors.forEach { sum += it.encoder.getRawTicks() }
        return sum / 2
    }

    fun rightRawTicks():Int{
        var sum = 0
        rightMotors.forEach { sum += it.encoder.getRawTicks() }
        return sum / 2
    }

    fun leftRawRadians():Double{
        var sum = 0.0
        leftMotors.forEach { sum += it.encoder.getRawRadians() }
        return sum / 2
    }

    fun rightRawRadians():Double{
        var sum = 0.0
        leftMotors.forEach { sum += it.encoder.getRawRadians() }
        return sum / 2
    }

    fun leftTicks():Int{
        var sum = 0
        leftMotors.forEach { sum += it.encoder.getTicks() }
        return sum / 2
    }

    fun rightTicks():Int{
        var sum = 0
        rightMotors.forEach { sum += it.encoder.getTicks() }
        return sum / 2
    }

    fun leftRadians():Double{
        var sum = 0.0
        leftMotors.forEach { sum += it.encoder.getRadians() }
        return sum / 2
    }

    fun rightRadians():Double{
        var sum = 0.0
        leftMotors.forEach { sum += it.encoder.getRadians() }
        return sum / 2
    }


    fun resetRightEncoders(){
        rightMotors.forEach { it.encoder.reset() }
    }
    fun resetLeftEncoders(){
        leftMotors.forEach { it.encoder.reset() }
    }

    fun resetEncoders(){
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

    override fun setMotorPowers(left: Double, right: Double) {
        leftMotors.forEach { it.power = left }
        rightMotors.forEach { it.power = right }
    }

    override fun getRawPos() = poseEstimate
    override fun updatePos() = updatePoseEstimate()

    private var activeTrajectoryFollower: TrajectoryFollower? = null

    override fun update() {
        imu.clearCaches()
        localizerArg.updatePos()
        when (controlState) {
            ControlLoopStates.FOLLOW_TRAJECTORY -> {
                val follower = activeTrajectoryFollower
                follower?.update(localizerArg.getPos())
            }
            ControlLoopStates.OPEN_LOOP   -> {
                val powers = openLoopWheelPowers.copy()
                setMotorPowers(powers.l, powers.r)
            }
            ControlLoopStates.DRIVING_AT_ANGLE -> {
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

    override fun start() {
    }

    fun stop() = openLoopPowerWheels(0.0, 0.0)

    fun startDrivingAtAngle(controller:PIDController = PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients()), power:Double = 1.0, angle: Double){
        followAngleData = FollowAngleData(controller = controller, power = power, angle = MathUtil.norm(angle, AngleUnit.DEGREES))
        controlState = ControlLoopStates.DRIVING_AT_ANGLE
    }

    fun startFollowingTrajectory(trajectory: Trajectory, followType: Follower = followerType) {
        val follower = getFollower(followType)
        follower.followTrajectory(trajectory)
        setActiveTrajectoryFollower(follower)
    }

    fun followingTrajectory(): Boolean = activeTrajectoryFollower?.isFollowing() ?: false

    fun setActiveTrajectoryFollower(follower: TrajectoryFollower) {
        controlState = ControlLoopStates.FOLLOW_TRAJECTORY
        activeTrajectoryFollower = follower
    }

    fun openLoopPowerWheels(l: Double, r: Double) {
        controlState = ControlLoopStates.OPEN_LOOP
        openLoopWheelPowers = SidePowers(l = l, r = r)
    }

    var openLoopWheelPowers = SidePowers(l = 0.0, r = 0.0)

    data class SidePowers(val l: Double, val r: Double)
}
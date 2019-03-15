package com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedDcMotorEx
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.odometry.Localizer
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.david.rechargedkotlinlibrary.internal.util.MathUtil
import com.qualcomm.robotcore.hardware.DcMotor
import java.util.*
import kotlin.math.abs

/**
 * Created by David Lukens on 8/2/2018.
 */
abstract class MecDrive(
        private val robot: RobotTemplate,
        private val lf: CachedDcMotorEx,
        private val lb: CachedDcMotorEx,
        private val rf: CachedDcMotorEx,
        private val rb: CachedDcMotorEx,
        mode: DcMotor.RunMode = DcMotor.RunMode.RUN_USING_ENCODER,
        zeroPowerBehavior: DcMotor.ZeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE,
        private val ENCODER_SCALER: Double = 1.0,
        private val RADIUS: Double = 2.0,
        AXIAL_PID_COEFFICIENTS: PIDCoefficients = PIDCoefficients(),
        TURN_PID_COEFFICIENTS: PIDCoefficients = PIDCoefficients(),
        kA: Double = 0.0,
        kV: Double = 0.0,
        kStatic: Double = 0.0,
        var MAX_VEL: Double = 1.0 / kV,
        val MAX_ACCEL: Double = 0.0,
        val MAX_TURN_ACCEL: Double = 0.0,
        localizerArg: Localizer? = null,
        TRACK_WIDTH: Double = 0.0,
        WHEEL_BASE: Double = 0.0)
    : MecanumDrive(TRACK_WIDTH, WHEEL_BASE), MTSubsystem, Localizer {
    private val HARD_MAX_VEL: Double = 1.0 / kV
    var posBias = Pose2d(Vector2d(0.0, 0.0), 0.0)
    private val localizerArg = localizerArg ?: this

    init {
        MAX_VEL = Math.min(HARD_MAX_VEL, MAX_VEL)
        lf.mode = mode
        lb.mode = mode
        rf.mode = mode
        rb.mode = mode
        lf.zeroPowerBehavior = zeroPowerBehavior
        lb.zeroPowerBehavior = zeroPowerBehavior
        rf.zeroPowerBehavior = zeroPowerBehavior
        rb.zeroPowerBehavior = zeroPowerBehavior
        robot.thread.addSubsystem(this)
    }

    private val baseConstraints = DriveConstraints(1.0 / kV, MAX_VEL, MAX_ACCEL, MAX_TURN_ACCEL)
    val hardConstraints = MecanumConstraints(baseConstraints, trackWidth, wheelBase)
    val follower = MecanumPIDVAFollower(this, AXIAL_PID_COEFFICIENTS, TURN_PID_COEFFICIENTS, kA = kA, kV = kV, kStatic = kStatic)
    @Throws(InterruptedException::class)
    fun waitOnFollower(condition: () -> Boolean = { true }, action: Runnable? = null) {
        while (robot.opMode.opModeIsActive() && follower.isFollowing() && condition()) {
            follower.update(localizerArg.getPos())
            action?.run()
        }
    }
    @Throws(InterruptedException::class)
    fun waitOnTrajectory(condition: () -> Boolean = { true }, action: Runnable? = null, trajectory: Trajectory) {
        follower.followTrajectory(trajectory)
        waitOnFollower(condition, action)
    }
    @Throws(InterruptedException::class)
    fun trajectoryBuilder(pos: Pose2d = localizerArg.getPos(), constraints: MecanumConstraints = hardConstraints) = TrajectoryBuilder(pos, constraints)

    @Throws(InterruptedException::class)
    fun powerTranslation(forward: Double, strafeRight: Double, turnClockwise: Double) = setMotorPowers(forward + strafeRight + turnClockwise, forward - strafeRight + turnClockwise, forward + strafeRight - turnClockwise, forward - strafeRight - turnClockwise)
    @Throws(InterruptedException::class)
    override fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
        val max = Collections.max(listOf(abs(frontLeft), abs(rearLeft), abs(frontRight), abs(rearRight), 1.0))
        lf.power = frontLeft / max
        lb.power = rearLeft / max
        rf.power = frontRight / max
        rb.power = rearRight / max
    }
    @Throws(InterruptedException::class)
    fun radiansToInches(radians: Double) = MathUtil.radiansToInches(radians * ENCODER_SCALER, RADIUS)
    @Throws(InterruptedException::class)
    override fun getWheelPositions(): List<Double> {
        val positions = LinkedList<Double>()
        positions.add(radiansToInches(lfRawRadians()))
        positions.add(radiansToInches(lbRawRadians()))
        positions.add(radiansToInches(rfRawRadians()))
        positions.add(radiansToInches(rbRawRadians()))
        return positions
    }
    @Throws(InterruptedException::class)
    fun resetEncoders() {
        resetLFEncoder()
        resetLBEncoder()
        resetRFEncoder()
        resetRBEncoder()
    }
    @Throws(InterruptedException::class)
    fun resetLFEncoder() = lf.encoder.reset()
    @Throws(InterruptedException::class)
    fun resetLBEncoder() = lb.encoder.reset()
    @Throws(InterruptedException::class)
    fun resetRFEncoder() = rf.encoder.reset()
    @Throws(InterruptedException::class)
    fun resetRBEncoder() = rb.encoder.reset()
    @Throws(InterruptedException::class)
    fun lfTicks() = lf.currentPosition
    @Throws(InterruptedException::class)
    fun lbTicks() = lb.currentPosition
    @Throws(InterruptedException::class)
    fun rfTicks() = rf.currentPosition
    @Throws(InterruptedException::class)
    fun rbTicks() = rb.currentPosition
    @Throws(InterruptedException::class)
    fun lfRawTicks() = lf.encoder.getRawTicks()
    @Throws(InterruptedException::class)
    fun lbRawTicks() = lb.encoder.getRawTicks()
    @Throws(InterruptedException::class)
    fun rfRawTicks() = rf.encoder.getRawTicks()
    @Throws(InterruptedException::class)
    fun rbRawTicks() = rb.encoder.getRawTicks()
    @Throws(InterruptedException::class)
    fun lfRadians() = lf.encoder.getRadians()
    @Throws(InterruptedException::class)
    fun lbRadians() = lb.encoder.getRadians()
    @Throws(InterruptedException::class)
    fun rfRadians() = rf.encoder.getRadians()
    @Throws(InterruptedException::class)
    fun rbRadians() = rb.encoder.getRadians()
    @Throws(InterruptedException::class)
    fun lfRawRadians() = lf.encoder.getRawRadians()
    @Throws(InterruptedException::class)
    fun lbRawRadians() = lb.encoder.getRawRadians()
    @Throws(InterruptedException::class)
    fun rfRawRadians() = rf.encoder.getRawRadians()
    @Throws(InterruptedException::class)
    fun rbRawRadians() = rb.encoder.getRawRadians()
    @Throws(InterruptedException::class)
    override fun update() = localizerArg.updatePos()
    @Throws(InterruptedException::class)
    override fun start() {
    }
    override var biasPose: Pose2d = Pose2d(0.0, 0.0, 0.0)
    @Throws(InterruptedException::class)
    override fun updatePos() = updatePoseEstimate()
    @Throws(InterruptedException::class)
    override fun getRawPos() = poseEstimate
}
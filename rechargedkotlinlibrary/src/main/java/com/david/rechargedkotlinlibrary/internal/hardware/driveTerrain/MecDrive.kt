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

    fun waitOnFollower(condition: () -> Boolean = { true }, action: Runnable? = null) {
        while (robot.opMode.opModeIsActive() && follower.isFollowing() && condition()) {
            follower.update(localizerArg.getPos())
            action?.run()
        }
    }

    fun waitOnTrajectory(condition: () -> Boolean = { true }, action: Runnable? = null, trajectory: Trajectory) {
        follower.followTrajectory(trajectory)
        waitOnFollower(condition, action)
    }

    fun trajectoryBuilder(pos: Pose2d = localizerArg.getPos(), constraints: MecanumConstraints = hardConstraints) = TrajectoryBuilder(pos, constraints)


    fun powerTranslation(forward: Double, strafeRight: Double, turnClockwise: Double) = setMotorPowers(forward + strafeRight + turnClockwise, forward - strafeRight + turnClockwise, forward + strafeRight - turnClockwise, forward - strafeRight - turnClockwise)

    override fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
        val max = Collections.max(listOf(abs(frontLeft), abs(rearLeft), abs(frontRight), abs(rearRight), 1.0))
        lf.power = frontLeft / max
        lb.power = rearLeft / max
        rf.power = frontRight / max
        rb.power = rearRight / max
    }

    fun radiansToInches(radians: Double) = MathUtil.radiansToInches(radians * ENCODER_SCALER, RADIUS)

    override fun getWheelPositions(): List<Double> {
        val positions = LinkedList<Double>()
        positions.add(radiansToInches(lfRawRadians()))
        positions.add(radiansToInches(lbRawRadians()))
        positions.add(radiansToInches(rfRawRadians()))
        positions.add(radiansToInches(rbRawRadians()))
        return positions
    }

    fun resetEncoders() {
        resetLFEncoder()
        resetLBEncoder()
        resetRFEncoder()
        resetRBEncoder()
    }

    fun resetLFEncoder() = lf.resetEncoder()
    fun resetLBEncoder() = lb.resetEncoder()
    fun resetRFEncoder() = rf.resetEncoder()
    fun resetRBEncoder() = rb.resetEncoder()
    fun lfTicks() = lf.currentPosition
    fun lbTicks() = lb.currentPosition
    fun rfTicks() = rf.currentPosition
    fun rbTicks() = rb.currentPosition
    fun lfRawTicks() = lf.getRawPosition()
    fun lbRawTicks() = lb.getRawPosition()
    fun rfRawTicks() = rf.getRawPosition()
    fun rbRawTicks() = rb.getRawPosition()
    fun lfRadians() = lf.getRadians()
    fun lbRadians() = lb.getRadians()
    fun rfRadians() = rf.getRadians()
    fun rbRadians() = rb.getRadians()
    fun lfRawRadians() = lf.getRawRadians()
    fun lbRawRadians() = lb.getRawRadians()
    fun rfRawRadians() = rf.getRawRadians()
    fun rbRawRadians() = rb.getRawRadians()

    override fun update() = localizerArg.updatePos()
    override fun start() {
    }

    override var biasPose: Pose2d = Pose2d(0.0, 0.0, 0.0)

    override fun updatePos() = updatePoseEstimate()

    override fun getRawPos() = poseEstimate
}
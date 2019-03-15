package org.firstinspires.ftc.teamcode.iterative.testing.bot

import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.iterative.lib.subsystems.SubsystemManager
import org.firstinspires.ftc.teamcode.iterative.lib.subsystems.Updatable
import kotlin.math.absoluteValue

/**
 * Created by David Lukens on 2/15/2019.
 */
class IterativeDrive : Updatable, MecanumDrive(DriveConstants.trackWidth) {

    data class DriveSignals(var lf: Double, var lb: Double, var rf: Double, var rb: Double) {
        @Throws(InterruptedException::class)
        fun norm(): DriveSignals {
            val max = listOf(lf.absoluteValue, lb.absoluteValue, rf.absoluteValue, rb.absoluteValue, 1.0).max()
                    ?: 1.0
            lf /= max
            lb /= max
            rf /= max
            rb /= max
            return this
        }
    }

    var driveSignals = DriveSignals(0.0, 0.0, 0.0, 0.0)

    private lateinit var lf: IterativeCachedDcMotorEx
    private lateinit var lb: IterativeCachedDcMotorEx
    private lateinit var rf: IterativeCachedDcMotorEx
    private lateinit var rb: IterativeCachedDcMotorEx
    private val motors = ArrayList<IterativeCachedDcMotorEx>()

    @Throws(InterruptedException::class)
    fun initHardware(hMap: HardwareMap, autonomous: Boolean, subsystemManager: SubsystemManager) {
        lf = IterativeCachedDcMotorEx(HardwareMaker.DcMotorEx.make(hMap, "lf"), IterativeBot.leftHub)
        lb = IterativeCachedDcMotorEx(HardwareMaker.DcMotorEx.make(hMap, "lb"), IterativeBot.leftHub)
        rf = IterativeCachedDcMotorEx(HardwareMaker.DcMotorEx.make(hMap, "rf", DcMotorSimple.Direction.REVERSE), IterativeBot.rightHub)
        rb = IterativeCachedDcMotorEx(HardwareMaker.DcMotorEx.make(hMap, "rb", DcMotorSimple.Direction.REVERSE), IterativeBot.rightHub)
        motors.add(lf)
        motors.add(lb)
        motors.add(rf)
        motors.add(rb)
        if (autonomous)
            motors.forEach { it.mode = DcMotor.RunMode.RUN_USING_ENCODER }

        constraints = MecanumConstraints(DriveConstants.baseConstraints, DriveConstants.trackWidth)
        trajectoryFollower = MecanumPIDVAFollower(this, DriveConstants.translationalPID, DriveConstants.headingPID, DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic)

        driveSignals = DriveSignals(0.0, 0.0, 0.0, 0.0)
        subsystemManager.addUpdatable(this)
    }

    @Throws(InterruptedException::class)
    override fun update() {
        updatePoseEstimate()
        val signals = driveSignals.copy().norm()
        lf.power = signals.lf
        lb.power = signals.lb
        rf.power = signals.rf
        rb.power = signals.rb
    }

    private lateinit var constraints: DriveConstraints
    lateinit var trajectoryFollower: TrajectoryFollower

    @Throws(InterruptedException::class)
    override fun getExternalHeading(): Double = IterativeBot.imu.getZ(AngleUnit.RADIANS)

    @Throws(InterruptedException::class)
    override fun getWheelPositions(): List<Double> = listOf(
            DriveConstants.encoderTicksToInches(lf.encoder.getRawTicks()),
            DriveConstants.encoderTicksToInches(lb.encoder.getRawTicks()),
            DriveConstants.encoderTicksToInches(rb.encoder.getRawTicks()),
            DriveConstants.encoderTicksToInches(rf.encoder.getRawTicks())
    )

    @Throws(InterruptedException::class)
    override fun setMotorPowers(lf: Double, lb: Double, rb: Double, rf: Double) {
        driveSignals = DriveSignals(lf, lb, rf, rb)
    }

    @Throws(InterruptedException::class)
    fun trajectoryBuilder() = TrajectoryBuilder(poseEstimate, constraints)

    @Throws(InterruptedException::class)
    fun stop() {
        driveSignals = DriveSignals(0.0, 0.0, 0.0, 0.0)
    }

    @Throws(InterruptedException::class)
    fun robotCentric(forward: Double, right: Double, clockwise: Double) {
        val lf = forward + right + clockwise
        val lb = forward - right + clockwise
        val rf = forward - right - clockwise
        val rb = forward + right - clockwise
        setMotorPowers(lf, lb, rb, rf)
    }

    @Throws(InterruptedException::class)
    fun tank(left: Double, right: Double) = setMotorPowers(left, left, right, right)

    @Throws(InterruptedException::class)
    fun tankArcade(x: Double, clockwise: Double) = tank(x + clockwise, x - clockwise)
}
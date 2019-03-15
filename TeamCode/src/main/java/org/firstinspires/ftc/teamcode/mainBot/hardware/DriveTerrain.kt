package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.acmerobotics.dashboard.config.Config
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
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.teamcode.mpTuningBot.DriveConstants
import kotlin.math.absoluteValue

@Config
class DriveTerrain(val robot: RobotTemplate) : DiffDrive(
        robot = robot,
        leftMotors = arrayOf(
                CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "lf"), robot.getHub(0)),
                CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "lb"), robot.getHub(0))
        ),
        rightMotors = arrayOf(
                CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "rf", direction = DcMotorSimple.Direction.REVERSE), robot.getHub(1)),
                CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "rb", direction = DcMotorSimple.Direction.REVERSE), robot.getHub(1))
        ),
        CROSSTRACK_PID = PIDCoefficients(0.0, 0.0, 0.0),
        DISPLACEMENT_PID = PIDCoefficients(0.0, 0.0, 0.0),
        imu = SimplifiedBNO055(HardwareMaker.BNO055IMU.make(robot.getHub(1).delegate, 0, true, BNO055IMU.SensorMode.IMU), AxesOrder.XZX),
        encoderTicksToInches = { ticks -> DriveConstants.encoderTicksToInches(ticks) },
        baseConstraints = DriveConstants.BASE_CONSTRAINTS
) {
    companion object {
        @JvmField
        var maxAcceleration: Double = 10.0
        @JvmField
        var kV: Double = 0.01747// calculated with 4.77 fps
        @JvmField
        var mpAnglePID = com.qualcomm.robotcore.hardware.PIDCoefficients(0.0, 0.0, 0.0)

        private const val ticksPerMotorRev: Double = 560.0
        private const val RADIUS: Double = 2.0
        private const val TICKS_PER_REV: Double = ticksPerMotorRev * (40.0 / 42.0)
    }

    private var lastLeftTicks = 0
    private var lastRightTicks = 0

    @Throws(InterruptedException::class)
    override fun getDistanceUpdate(): Double {
        val leftTicks = leftRawTicks()
        val rightTicks = rightRawTicks()
        val leftInches = toInches(leftTicks - lastLeftTicks)
        val rightInches = toInches(rightTicks - lastRightTicks)
        lastLeftTicks = leftTicks
        lastRightTicks = rightTicks
        return (rightInches + leftInches) / 2.0
    }

    @Throws(InterruptedException::class)
    fun toInches(ticks: Int): Double = RADIUS * 2.0 * Math.PI * ticks.toDouble() / TICKS_PER_REV

    private enum class FollowingLineState {
        OVER,
        UNDER
    }

    @Throws(InterruptedException::class)
    fun drive(inches: Double, angle: Double, maxVel: Double = 40.0, maxAccel: Double = maxAcceleration, threshold: Double = 1.0) {
        mp(inches, angle, maxVel, maxAccel, kV, threshold, PIDController(mpAnglePID))
        robot.opMode.waitWhile { isMP() }

        /*val angleController = PIDController(mpAnglePID)
        getDistanceUpdate()
        var inchesTraveled = 0.0
        var vel = 0.0
        val timer = DeltaTimer()

        robot.opMode.loopWhile({ (inches - inchesTraveled).absoluteValue > threshold }, {
            inchesTraveled += getDistanceUpdate()

            var accel:Double
            var state = FollowingLineState.UNDER
            val dt = timer.seconds()
            val distanceLeft = inches - inchesTraveled

            val timeToStop = vel.absoluteValue / maxAcceleration
            val distanceToStop = vel.absoluteValue * timeToStop + 0.5 * + -maxAcceleration * timeToStop.pow(2)

            if (distanceLeft.sign * vel.sign < -0.1)
                state = FollowingLineState.OVER
            if (state == FollowingLineState.OVER || distanceLeft.absoluteValue <= distanceToStop.absoluteValue)
                accel = dt * maxAcceleration * -distanceLeft.sign // todo figure out how to determine if full acceleration is too much
            else
                accel = dt * maxAcceleration * distanceLeft.sign // todo figure out how to determine if full acceleration is too much


            vel += accel
            vel = Range.clip(vel, -maxVel, maxVel)
            val xVel = vel * kV
            val headingVel = angleController.update(MathUtil.norm(angle - imu.getZ(), AngleUnit.DEGREES))

            robot.opMode.telemetry.addData("xvel", xVel)
            robot.opMode.telemetry.addData("headingVel", headingVel)
            robot.opMode.telemetry.addData("inchesTraveled", inchesTraveled)
            robot.opMode.telemetry.addData("distanceLeft", distanceLeft)
            robot.opMode.telemetry.addData("rightRawTicks", rightRawTicks())
            robot.opMode.telemetry.addData("leftRawTicks", leftRawTicks())
            robot.opMode.telemetry.addData("state", state)
            robot.opMode.telemetry.addData("accel", accel)
            robot.opMode.telemetry.addData("vel", vel)
            robot.opMode.telemetry.addData("mavVel", maxVel)
            robot.opMode.telemetry.addData("maxAcceleration", maxAcceleration)
            robot.opMode.telemetry.update()

            openLoopArcade(vel * kV, headingVel)
        })
        stop()*/
    }

    enum class AngleFollowSpeeds(val controller: PIDController, val speed: Double) {
        LINE_DETECT(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients(0.005, 0.0, 0.001)), 0.1),// 0.05 speed works // 0.15 works almost always
        PARK(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients(0.005, 0.0, 0.0005)), 0.15),
        FAST(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients(0.005, 0.0, 0.0005)), 1.0),
        SLOW(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients(0.005, 0.0, 0.0005)), 0.3),
        HALF(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients(0.005, 0.0, 0.0005)), 0.5),
        // 0.005, 0.0, 0.001 at lm1
        // 0.01, 0.0, 0.0013 at lm3
        TURN(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients(0.005, 0.0, 0.001)), 0.0),
        STRAFE(PIDController(com.qualcomm.robotcore.hardware.PIDCoefficients(0.005, 0.0, 0.001)), 0.0)
    }

    @Throws(InterruptedException::class)
    fun startFollowingAngle_setConstants(angleFollowSpeed: AngleFollowSpeeds = AngleFollowSpeeds.FAST, angle: Double, reverse: Boolean = false, type: AnglePIDType, maxTurnPower: Double = 1.0) {
        startFollowingAngle(angleFollowSpeed.controller, if (reverse) -angleFollowSpeed.speed else angleFollowSpeed.speed, angle, type, maxTurnPower)
    }

    @Throws(InterruptedException::class)
    fun pidTurn(target: Double, threshold: Double = 2.0, stop: Boolean = true, maxTurnPower: Double = 1.0) {
        startFollowingAngle_setConstants(AngleFollowSpeeds.TURN, target, false, AnglePIDType.POINT_TURN, maxTurnPower)
        robot.opMode.waitTill { (imu.getZ(AngleUnit.DEGREES) - target).absoluteValue < threshold }
        if (stop)
            stop()
    }

    @Throws(InterruptedException::class)
    fun deadReckonPID(ticks: Int, angle: Double, speed: AngleFollowSpeeds = AngleFollowSpeeds.FAST, stop: Boolean = true) {
        if (ticks != 0) {
            val reverse = ticks < 0
            resetEncoders()
            startFollowingAngle_setConstants(speed, angle, reverse, AnglePIDType.STRAIGHT)
            if (reverse)
                robot.opMode.waitTill { (leftTicks() + rightTicks()) < ticks }
            else
                robot.opMode.waitTill { (leftTicks() + rightTicks()) > ticks }
        }
        if (stop)
            stop()
    }

    @Throws(InterruptedException::class)
    fun strafeAroundLeft(target: Double, threshold: Double = 2.0, stop: Boolean = true, maxTurnPower: Double = 1.0) {
        startFollowingAngle_setConstants(AngleFollowSpeeds.STRAFE, target, false, AnglePIDType.TURN_AROUND_LEFT, maxTurnPower)
        robot.opMode.waitTill { (imu.getZ(AngleUnit.DEGREES) - target).absoluteValue < threshold }
        if (stop)
            stop()
    }

    @Throws(InterruptedException::class)
    fun strafeAroundRight(target: Double, threshold: Double = 2.0, stop: Boolean = true, maxTurnPower: Double = 1.0) {
        startFollowingAngle_setConstants(AngleFollowSpeeds.STRAFE, target, false, AnglePIDType.TURN_AROUND_RIGHT, maxTurnPower)
        robot.opMode.waitTill { (imu.getZ(AngleUnit.DEGREES) - target).absoluteValue < threshold }
        if (stop)
            stop()
    }

    @Throws(InterruptedException::class)
    fun runTime(power: Double, seconds: Double, stop: Boolean = true) {
        if (seconds != 0.0) {
            openLoopArcade(power)
            robot.opMode.sleepSeconds(seconds)
        }
        if (stop)
            stop()
    }
}
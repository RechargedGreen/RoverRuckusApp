package com.david.rechargedkotlinlibrary.internal.roadRunner

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.drive.Drive
import com.acmerobotics.roadrunner.drive.TankKinematics
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import com.david.rechargedkotlinlibrary.internal.opMode.RechargedLinearOpMode
import org.apache.commons.math3.stat.regression.SimpleRegression
import java.util.*

/**
 * Op mode for computing kV, kStatic, and kA from various drive routines.
 */
abstract class FeedforwardTuningOpMode<rt : RobotTemplate>
/**
 * @param distance allowable forward travel distance
 * @param wheelMotorRpm wheel motor rpm
 * @param wheelDiameter wheel diameter
 * @param wheelGearRatio wheel gear ratio (output / input)
 */
@JvmOverloads constructor(createRobot: (RechargedLinearOpMode<rt>) -> rt, private val distance: Double) : FluidAuto<rt>(createRobot) {
    @Throws(InterruptedException::class)
    override fun run() {
        val d = robot.getDrive()
        var wheelMotorRpm = d.getMaxWheelMotorRPM()
        var wheelDiameter = d.getWheelRadius() * 2.0
        val wheelGearRatio = d.getWheelGearRatio()
        val drive = d.getDrive()

        fun setVelocity(vel: Pose2d){
            val powers = TankKinematics.robotToWheelVelocities(vel, 0.0)
            d.setVel(vel)
        }

        telemetry.log().add("Press play to begin the feedforward tuning routine")
        telemetry.update()

        waitForStart()

        telemetry.log().clear()
        telemetry.log().add("Would you like to fit kStatic?")
        telemetry.log().add("Press (A) for yes, (B) for no")
        telemetry.update()

        var fitIntercept = false
        while (opModeIsActive()) {
            if (gamepad1.a) {
                fitIntercept = true
                while (opModeIsActive() && gamepad1.a);
                break
            } else if (gamepad1.b) {
                while (opModeIsActive() && gamepad1.b);
                break
            }
        }

        telemetry.log().clear()
        telemetry.log().add(String.format("Place your robot on the field with at least %.2f in of room in front", distance))
        telemetry.log().add("Press (A) to begin")
        telemetry.update()

        while (opModeIsActive() && !gamepad1.a);
        while (opModeIsActive() && gamepad1.a);

        telemetry.log().clear()
        telemetry.log().add("Running...")
        telemetry.update()

        val maxVel = wheelMotorRpm * wheelGearRatio * Math.PI * wheelDiameter / 60.0
        val finalVel = MAX_POWER * maxVel
        val accel = finalVel * finalVel / (2.0 * distance)
        val rampTime = Math.sqrt(2.0 * distance / accel)

        var startTime = System.nanoTime() / 1e9
        val timeSamples = ArrayList<Double>()
        val powerSamples = ArrayList<Double>()
        val positionSamples = ArrayList<Double>()

        drive.poseEstimate = Pose2d()
        while (opModeIsActive()) {
            val elapsedTime = System.nanoTime() / 1e9 - startTime
            if (elapsedTime > rampTime)
                break
            val vel = accel * elapsedTime
            val power = vel / maxVel

            timeSamples.add(elapsedTime)
            powerSamples.add(power)
            positionSamples.add(drive.poseEstimate.x)

            setVelocity(Pose2d(power, 0.0, 0.0))
            //drive.updatePoseEstimate()
        }
        setVelocity(Pose2d(0.0, 0.0, 0.0))

        var velocitySamples = numericalDerivative(timeSamples, positionSamples)
        val rampRegression = SimpleRegression(fitIntercept)
        for (i in velocitySamples.indices)
            rampRegression.addData(velocitySamples[i], powerSamples[i])
        val kV = rampRegression.slope
        val kStatic = rampRegression.intercept

        telemetry.log().clear()
        telemetry.log().add("Quasi-static ramp up test complete")
        if (fitIntercept)
            telemetry.log().add(String.format("kV = %.5f, kStatic = %.5f (R^2 = %.2f)", kV, kStatic, rampRegression.rSquare))
        else
            telemetry.log().add(String.format("kV = %.5f (R^2 = %.2f)", kV, rampRegression.rSquare))
        telemetry.log().add("Would you like to fit kA?")
        telemetry.log().add("Press (A) for yes, (B) for no")
        telemetry.update()

        var fitAccelFF = false
        while (opModeIsActive()) {
            if (gamepad1.a) {
                fitAccelFF = true
                while (opModeIsActive() && gamepad1.a);
                break
            } else if (gamepad1.b) {
                while (opModeIsActive() && gamepad1.b);
                break
            }
        }

        if (fitAccelFF) {
            telemetry.log().clear()
            telemetry.log().add("Place the robot back in its starting position")
            telemetry.log().add("Press (A) to continue")
            telemetry.update()

            while (opModeIsActive() && !gamepad1.a);
            while (opModeIsActive() && gamepad1.a);

            telemetry.log().clear()
            telemetry.log().add("Running...")
            telemetry.update()

            val maxPowerTime = distance / maxVel

            startTime = System.nanoTime() / 1e9
            timeSamples.clear()
            positionSamples.clear()

            drive.poseEstimate = Pose2d()
            setVelocity(Pose2d(MAX_POWER, 0.0, 0.0))
            while (opModeIsActive()) {
                val elapsedTime = System.nanoTime() / 1e9 - startTime
                if (elapsedTime > maxPowerTime) {
                    break
                }

                timeSamples.add(elapsedTime)
                positionSamples.add(drive.poseEstimate.x)

                //drive.updatePoseEstimate()
            }
            setVelocity(Pose2d(0.0, 0.0, 0.0))

            velocitySamples = numericalDerivative(timeSamples, positionSamples)
            val accelerationSamples = numericalDerivative(timeSamples, velocitySamples)

            val maxPowerRegresiion = SimpleRegression(false)
            for (i in accelerationSamples.indices) {
                var velocityPower = kV * velocitySamples[i]
                if (Math.abs(velocityPower) > EPSILON) {
                    velocityPower += Math.signum(velocityPower) * kStatic
                } else {
                    velocityPower = 0.0
                }
                val accelerationPower = MAX_POWER - velocityPower
                maxPowerRegresiion.addData(accelerationSamples[i], accelerationPower)
            }
            val kA = maxPowerRegresiion.slope

            telemetry.log().clear()
            telemetry.log().add("Max power test complete")
            telemetry.log().add(String.format("kA = %.5f (R^2 = %.2f)", kA, maxPowerRegresiion.rSquare))
            telemetry.update()
        }

        while (opModeIsActive()) {
            idle()
        }
    }

    companion object {
        private val MAX_POWER = 0.7
        private val EPSILON = 1e-2

        private fun numericalDerivative(x: List<Double>, y: List<Double>): List<Double> {
            val deriv = ArrayList<Double>()
            for (i in 2 until x.size) {
                deriv.add((y[i] - y[i - 2]) / (x[i] - x[i - 2]))
            }
            deriv.add(0, deriv[0])
            deriv.add(deriv[deriv.size - 1])
            return deriv
        }
    }

}
package com.david.rechargedkotlinlibrary.internal.roadRunner

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.drive.TankKinematics
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import com.david.rechargedkotlinlibrary.internal.opMode.RechargedLinearOpMode
import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

/**
 * Op mode for measuring the empirical track width of a robot drive.
 */
abstract class TrackWidthCalibrationOpMode<rt : RobotTemplate>
/**
 * @param totalRevolutions number of revolutions
 * @param power angular power
 */
@JvmOverloads constructor(createRobot: (RechargedLinearOpMode<rt>) -> rt, private val totalRevolutions: Int = 1, private val power: Double = 0.3) : FluidAuto<rt>(createRobot) {

    @Throws(InterruptedException::class)
    override fun run() {
        val d = robot.getDrive()
        val imu = d.getGyro()
        val drive = d.getDrive()

        fun setVelocity(vel: Pose2d) {
            val powers = TankKinematics.robotToWheelVelocities(vel, 0.0)
            d.setVel(vel)
        }

        telemetry.log().add("Press play to begin the track width calibration routine")
        telemetry.log().add("Make sure your robot has enough clearance to turn smoothly")
        telemetry.log().add("Additionally, set the drive's track width to 1")
        telemetry.update()

        waitForStart()

        telemetry.log().clear()
        telemetry.log().add("Running...")
        telemetry.update()

        var revolutions = 0
        var startedMoving = false
        var lastHeading = 0.0

        drive.poseEstimate = Pose2d()
        setVelocity(Pose2d(0.0, 0.0, power))
        while (opModeIsActive() && (!startedMoving || revolutions <= totalRevolutions)) {
            var heading = imu.getRawZ(AngleUnit.RADIANS)
            if (heading >= Math.PI / 2.0)
                startedMoving = true
            if (startedMoving && lastHeading < 0.0 && heading >= 0.0)
                revolutions++
            lastHeading = heading
        }
        setVelocity(Pose2d(0.0, 0.0, 0.0))
        val effectiveTrackWidth = drive.poseEstimate.heading / (2.0 * Math.PI * totalRevolutions.toDouble())

        telemetry.log().clear()
        telemetry.log().add("Calibration complete")
        telemetry.log().add(String.format("effective track width = %.2f", effectiveTrackWidth))
        telemetry.update()

        while (opModeIsActive());
    }
}

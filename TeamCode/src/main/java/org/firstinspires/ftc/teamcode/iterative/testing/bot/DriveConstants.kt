package org.firstinspires.ftc.teamcode.iterative.testing.bot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.david.rechargedkotlinlibrary.internal.util.MathUtil
import com.qualcomm.robotcore.hardware.PIDCoefficients

/**
 * Created by David Lukens on 2/15/2019.
 */
@Config
object DriveConstants {
    var pidConstants = PIDCoefficients(0.005, 0.0, 0.0005)

    var translationalPID = com.acmerobotics.roadrunner.control.PIDCoefficients()
    var headingPID = com.acmerobotics.roadrunner.control.PIDCoefficients()

    var kV = 0.0
    var kA = 0.0
    var kStatic = 0.0

    var baseConstraints = DriveConstraints(0.0, 0.0, 0.0, 0.0)

    val trackWidth = 1.0
    var wheelBase = 1.0
    var wheelRadius = 2.0
    var ticksPerRev = 560.0
    var gearRatio = 44.0 / 42.0
    var maxRPM = 300.0

    fun encoderTicksToInches(ticks: Int): Double = wheelRadius * MathUtil.TAU * gearRatio * ticks / ticksPerRev
    fun rpmToVelocity(rpm: Double) = rpm * gearRatio * MathUtil.TAU * wheelRadius / 60.0
}
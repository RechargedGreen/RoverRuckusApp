package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.util.Angle
import com.david.rechargedkotlinlibrary.internal.hardware.PIDController
import com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain.DiffDrive
import com.david.rechargedkotlinlibrary.internal.util.MathUtil
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.math.absoluteValue

@Config
open class PIDDriveSource(private val inches:Double, private val degrees:Double, private val maxSpeed:Double, private val threshold:Double, private val robot:HardwareClass) : DiffDrive.DriveSource(){
    companion object {
        @JvmField
        var driveKP:Double = 0.15
        @JvmField
        var driveKI:Double = 0.0
        @JvmField
        var driveKD:Double = 0.0

        @JvmField
        var turnKP:Double = 0.005
        @JvmField
        var turnKI:Double = 0.0
        @JvmField
        var turnKD:Double = 0.0005
    }

    private val turnPID = PIDController(PIDCoefficients(turnKP, turnKI, turnKD))
    private val drivePID = PIDController(PIDCoefficients(driveKP, driveKI, driveKD))

    init {
        robot.drive.resetEncoders()
    }

    private fun distanceTraveled() = robot.drive.toInches(robot.drive.leftTicks() + robot.drive.rightTicks()) / 2.0

    override fun getVel(): DiffDrive.Velocity = DiffDrive.Velocity(Range.clip(drivePID.update(error()), -maxSpeed, maxSpeed), -turnPID.update(angleOff()))

    private fun error() = inches - distanceTraveled()

    private fun angleOff() = MathUtil.norm(degrees - robot.drive.imu.getZ(AngleUnit.DEGREES), AngleUnit.DEGREES)

    override fun isComplete():Boolean = error().absoluteValue < threshold
}
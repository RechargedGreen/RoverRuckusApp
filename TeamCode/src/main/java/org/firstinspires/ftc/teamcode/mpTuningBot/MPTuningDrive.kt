package org.firstinspires.ftc.teamcode.mpTuningBot

import com.acmerobotics.roadrunner.drive.TankDrive
import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

class MPTuningDrive (hMap: HardwareMap) : TankDrive(1.0) {
    private val lf = hMap.get(DcMotorEx::class.java, "lf")
    private val lb = hMap.get(DcMotorEx::class.java, "lb")
    private val rf = hMap.get(DcMotorEx::class.java, "rf")
    private val rb = hMap.get(DcMotorEx::class.java, "rb")

    init {
        rf.direction = DcMotorSimple.Direction.REVERSE
        rb.direction = DcMotorSimple.Direction.REVERSE
    }

    val imu = HardwareMaker.BNO055IMU.make(hMap, "imu", true, BNO055IMU.SensorMode.IMU)

    override fun getWheelPositions() = listOf(
            DriveConstants.encoderTicksToInches(lf.currentPosition),
            DriveConstants.encoderTicksToInches(rf.currentPosition))

    override fun setMotorPowers(left: Double, right: Double) {
        lf.power = left
        lb.power = left
        rf.power = right
        rb.power = right
    }

    fun getExternalHeading() = imu.angularOrientation.firstAngle.toDouble()

    override fun getHeading() = getExternalHeading()
}
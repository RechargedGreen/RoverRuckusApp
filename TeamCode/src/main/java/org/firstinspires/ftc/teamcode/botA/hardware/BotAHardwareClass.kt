package org.firstinspires.ftc.teamcode.botA.hardware

import com.acmerobotics.roadrunner.drive.Drive
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.david.rechargedkotlinlibrary.internal.opMode.RechargedLinearOpMode
import com.qualcomm.hardware.bosch.BNO055IMU

class BotAHardwareClass(opMode:RechargedLinearOpMode<BotAHardwareClass>) : RobotTemplate(opMode, arrayOf("leftHub")){
    val imu = hMap.get(BNO055IMU::class.java, "imu")
    val drive = BotADrive(this)
    val superSystem = BotASuperSystem(this)

    override fun autoPostInit() = null!!
    override fun getMaxWheelMotorRPM(): Double = drive.MOTOR_TYPE.ticksPerRev / drive.WHEEL_GEAR_RATIO
    override fun getWheelRadius(): Double = drive.RADIUS
    override fun getWheelGearRatio(): Double = drive.WHEEL_GEAR_RATIO
    override fun getDrive(): Drive  = drive
    override fun getGyro(): BNO055IMU = imu
}
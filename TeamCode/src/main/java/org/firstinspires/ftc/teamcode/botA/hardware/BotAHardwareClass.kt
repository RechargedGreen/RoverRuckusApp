package org.firstinspires.ftc.teamcode.botA.hardware

import com.acmerobotics.roadrunner.drive.Drive
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.david.rechargedkotlinlibrary.internal.opMode.RechargedLinearOpMode
import com.david.rechargedkotlinlibrary.internal.util.AutoTransitionerKotlin
import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.teamcode.botA.teleOp.BotATeleOp

class BotAHardwareClass(opMode:RechargedLinearOpMode<BotAHardwareClass>) : RobotTemplate(opMode, arrayOf("leftHub")){
    val imu = hMap.get(BNO055IMU::class.java, "imu")
    val drive = BotADrive(this)
    val superSystem = BotASuperSystem(this)
    val dumper = Dumper(this)
    val intake = Intake(this)

    override fun autoPostInit() = AutoTransitionerKotlin.transitionOnStop(opMode, BotATeleOp.NAME)
    override fun getMaxWheelMotorRPM(): Double = drive.MOTOR_TYPE.ticksPerRev
    override fun getWheelRadius(): Double = drive.RADIUS
    override fun getWheelGearRatio(): Double = drive.WHEEL_GEAR_RATIO
    override fun getDrive(): Drive  = drive
    override fun getGyro(): BNO055IMU = imu
}
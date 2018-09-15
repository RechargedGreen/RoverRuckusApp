package org.firstinspires.ftc.teamcode.prototypes

import com.acmerobotics.roadrunner.drive.Drive
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.david.rechargedkotlinlibrary.internal.opMode.RechargedLinearOpMode
import com.david.rechargedkotlinlibrary.internal.util.AutoTransitionerKotlin
import com.qualcomm.hardware.bosch.BNO055IMU

/**
 * Created by David Lukens on 9/10/2018.
 */
class RR1Bot(opmode:RechargedLinearOpMode<RR1Bot>) : RobotTemplate(opmode, arrayOf("hub1")){
    val drive = RR1Drive(robot = this)
    override fun autoPostInit(){}// = AutoTransitionerKotlin.transitionOnStop(opMode, RR1Tele.NAME)
    override fun getDrive():Drive = drive
    override fun getMaxWheelMotorRPM() = drive.MOTOR_TYPE.maxRPM
    override fun getGyro() = hMap.get(BNO055IMU::class.java, "imu")
    override fun getWheelRadius() = drive.RADIUS
    override fun getWheelGearRatio() = drive.WHEEL_GEAR_RATIO
}
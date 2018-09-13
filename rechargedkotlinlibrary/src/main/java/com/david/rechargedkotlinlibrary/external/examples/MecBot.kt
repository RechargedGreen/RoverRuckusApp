package com.david.rechargedkotlinlibrary.external.examples

import com.acmerobotics.roadrunner.drive.Drive
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.david.rechargedkotlinlibrary.internal.opMode.RechargedLinearOpMode
import com.david.rechargedkotlinlibrary.internal.util.AutoTransitionerKotlin
import com.qualcomm.hardware.bosch.BNO055IMU

/**
 * Created by David Lukens on 8/2/2018.
 */

class MecBot(opMode: RechargedLinearOpMode<MecBot>) : RobotTemplate(opMode, arrayOf("hub1")) {
    val drive = ExampleMecDrive(this)
    override fun autoPostInit() = AutoTransitionerKotlin.transitionOnStop(opMode, MecTele.NAME)
    override fun getDrive(): Drive = drive
    override fun getGyro(): BNO055IMU = null!!
}
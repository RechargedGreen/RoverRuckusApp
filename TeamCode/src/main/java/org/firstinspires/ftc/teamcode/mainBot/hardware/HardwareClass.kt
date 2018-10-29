package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.acmerobotics.roadrunner.drive.Drive
import com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain.TunableDrive
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.david.rechargedkotlinlibrary.internal.opMode.RechargedLinearOpMode
import com.david.rechargedkotlinlibrary.internal.util.AutoTransitionerKotlin
import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.teamcode.mainBot.teleOp.Competition

class HardwareClass(opMode: RechargedLinearOpMode<HardwareClass>) : RobotTemplate(opMode, arrayOf("leftHub", "rightHub")) {
    val drive = DriveTerrain(this)
    val superSystem = SuperSystem(this)
    val dumper = Dumper(this)
    val intake = Intake(this)
    val lift = Lift(this)

    override fun autoPostInit() = AutoTransitionerKotlin.transitionOnStop(opMode, Competition.NAME)

    override fun getDrive():TunableDrive = drive
}
package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.acmerobotics.roadrunner.drive.Drive
import com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain.TunableDrive
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.david.rechargedkotlinlibrary.internal.opMode.RechargedLinearOpMode
import com.david.rechargedkotlinlibrary.internal.util.AutoTransitionerKotlin
import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.teamcode.mainBot.teleOp.Competition
import org.firstinspires.ftc.teamcode.vision.MasterVision

class HardwareClass(opMode: RechargedLinearOpMode<HardwareClass>) : RobotTemplate(opMode, arrayOf("leftHub", "rightHub")) {
    val drive = DriveTerrain(this)
    val superSystem = SuperSystem(this)
    val dumper = Dumper(this)
    val intake = Intake(this)
    val lift = Lift(this)
    val vision:MasterVision
    init {
        val parameters = VuforiaLocalizer.Parameters()
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK
        parameters.vuforiaLicenseKey = VUFORIA_KEY
        vision = MasterVision(parameters, hMap)
    }

    override fun autoPostInit() {
        AutoTransitionerKotlin.transitionOnStop(opMode, Competition.NAME)
        vision.enable()
    }
    override fun onPressingAutoPlay() = vision.stop()

    override fun getDrive():TunableDrive = drive

    companion object {
        const val VUFORIA_KEY =  "AXi/CxP/////AAAAGV4xMjmD2EwntmuvBtxZnj8AOji5oAG2lxjzOJIGA9IASLd1EtX7KzZ6BpH6J0FWgEcjd8O/6mWD1rvLoAZ1R3KJcxH/xss+scSbd/U8d7/cZDupryfSH7lbRv94ZmPPwduAaQOkxyZfX0Gv+IsMUtIGqTZ5WIHYpqRSHIsGQQ6nlslCi5x/NRu0tnV1t6YgX6svoenYGXpbktnCYZB5BwO7OTfw7XrMMWtqSCJrd3PZha8rgiN1VvqvdEok//H0d9Vh5pnAMa8XwMEXx0N/0V1uEGUEcQvQA+fK7zghPqxjiXBQoZxcUUGkSbNGaIfTPBEoNoOi8QzHo4N6QN1TrgLnJW9J6tgbz9xzTpnRahqU"
    }
}
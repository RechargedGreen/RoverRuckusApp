package org.firstinspires.ftc.teamcode.vision

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.teamcode.data.VisionConstants
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(group = OpModeGroups.TELE_MISC)
class VisionTest : LinearOpMode(){
    override fun runOpMode() {
        val parameters = VuforiaLocalizer.Parameters()
        parameters.vuforiaLicenseKey = VisionConstants.VUFORIA_KEY
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK

        val vision = MasterVision(parameters, hardwareMap)

        vision.init() // enables the camera overview. do at the beginning of the program as it takes a while

        vision.enable() // starts the algorithm

        waitForStart()

        while (opModeIsActive()){
            if (gamepad1.x)
                vision.disable()// disables the algorithm to free up processing power
            telemetry.addData("last known order", vision.tfLite.lastKnownSampleOrder)
            telemetry.update()
        }

        vision.shutdown() // shuts down detectors
    }
}
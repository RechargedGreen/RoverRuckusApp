package org.firstinspires.ftc.teamcode.vision

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(group = OpModeGroups.TELE_MISC)
class VisionTest : LinearOpMode(){
    override fun runOpMode() {
        val parameters = VuforiaLocalizer.Parameters()
        parameters.vuforiaLicenseKey = HardwareClass.VUFORIA_KEY
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK

        val vision = MasterVision(parameters, hardwareMap)

        vision.enable()

        waitForStart()

        while (opModeIsActive()){
            if (gamepad1.x)
                vision.disable()
            telemetry.addData("last known order", vision.tfLite.lastKnownSampleOrder)
            telemetry.update()
        }

        vision.shutdown()
    }
}
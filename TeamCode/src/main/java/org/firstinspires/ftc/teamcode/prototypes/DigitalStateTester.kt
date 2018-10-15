package org.firstinspires.ftc.teamcode.prototypes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(group = OpModeGroups.TELE_MISC)
class DigitalStateTester:LinearOpMode(){
    override fun runOpMode() {
        val d = hardwareMap.digitalChannel.get("d0")
        waitForStart()
        while (opModeIsActive()){
            telemetry.addData("state", d.state)
            telemetry.update()
        }
    }
}
package org.firstinspires.ftc.teamcode.mainBot.diagnostics

import com.david.rechargedkotlinlibrary.internal.opMode.PracticeTeleOp
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(group = OpModeGroups.TELE_DIAGNOSTICS)
class DriveEncodersTester : PracticeTeleOp<HardwareClass>({ opMode -> HardwareClass(opMode) }){
    override fun onLoop() {
        telemetry.addData("lf", robot.getHub(0).getEncoder(0))
        telemetry.addData("lb", robot.getHub(0).getEncoder(1))
        telemetry.addData("rf", robot.getHub(0).getEncoder(2))
        telemetry.addData("rb", robot.getHub(0).getEncoder(3))
    }
}
package org.firstinspires.ftc.teamcode.mainBot.diagnostics

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups
import org.firstinspires.ftc.teamcode.mainBot.teleOp.Practice

@TeleOp(group = OpModeGroups.TELE_DIAGNOSTICS)
class CompleteDiagnostics :Practice(){
    override fun onLoop() {
        super.onLoop()
        telemetry.addData("down limit", robot.lift.isFullyDown())
        telemetry.addData("up limit", robot.lift.isFullyUp())
        telemetry.addData("right wall distance", robot.sensors.getRightDistanceFromWall(0.0))
        telemetry.addData("reds", robot.sensors.lineDetector.reds)
        telemetry.addData("blues", robot.sensors.lineDetector.blues)
        telemetry.addData("On line", robot.sensors.lineDetector.onLine)
    }
}
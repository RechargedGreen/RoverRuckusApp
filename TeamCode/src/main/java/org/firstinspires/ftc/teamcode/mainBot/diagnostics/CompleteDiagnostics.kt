package org.firstinspires.ftc.teamcode.mainBot.diagnostics

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups
import org.firstinspires.ftc.teamcode.mainBot.teleOp.Practice

@TeleOp(group = OpModeGroups.TELE_DIAGNOSTICS)
class CompleteDiagnostics :Practice(){
    override fun onStart() {
        super.onStart()
        robot.sensors.lineDetector.enabled = true
    }
    override fun onLoop() {
        super.onLoop()
        telemetry.addData("down limit", robot.lift.isFullyDown())
        telemetry.addData("up limit", robot.lift.isFullyUp())
        telemetry.addData("right wall distance", robot.sensors.getRightDistanceFromWall(0.0))
        robot.sensors.lineDetector.reds.forEach { telemetry.addData("reds", it) }
        robot.sensors.lineDetector.blues.forEach { telemetry.addData("blues", it) }
        telemetry.addData("blues", robot.sensors.lineDetector.blues)
        telemetry.addData("On line", robot.sensors.lineDetector.onLine)
    }
}
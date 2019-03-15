package org.firstinspires.ftc.teamcode.mainBot.diagnostics

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups
import org.firstinspires.ftc.teamcode.mainBot.teleOp.PracticeJVoExtension

@TeleOp(group = OpModeGroups.TELE_DIAGNOSTICS)
class CompleteDiagnostics : PracticeJVoExtension() {
    @Throws(InterruptedException::class)
    override fun onStart() {
        super.onStart()
        robot.sensors.lineDetector.enabled = true
        robot.sensors.backIntoWallDetector.enabled = true
        robot.sensors.craterRimDetector.enabled = true
    }

    @Throws(InterruptedException::class)
    override fun onLoop() {
        super.onLoop()
        telemetry.addData("leftRed", robot.superSystem.bucketSense.leftRed)
        telemetry.addData("leftGreen", robot.superSystem.bucketSense.leftGreen)
        telemetry.addData("leftBlue", robot.superSystem.bucketSense.leftBlue)
        telemetry.addData("leftAlpha", robot.superSystem.bucketSense.leftAlpha)
        telemetry.addData("leftARGB", robot.superSystem.bucketSense.leftARGB)
        telemetry.addData("rightRed", robot.superSystem.bucketSense.rightRed)
        telemetry.addData("rightGreen", robot.superSystem.bucketSense.rightGreen)
        telemetry.addData("rightBlue", robot.superSystem.bucketSense.rightBlue)
        telemetry.addData("rightAlpha", robot.superSystem.bucketSense.rightAlpha)
        telemetry.addData("rightARGB", robot.superSystem.bucketSense.rightARGB)
        telemetry.addData("leftSensed", robot.superSystem.bucketSense.left())
        telemetry.addData("rightSensed", robot.superSystem.bucketSense.right())
        telemetry.addLine()
        telemetry.addData("close", robot.sensors.craterRimDetector.close())
        telemetry.addData("medium", robot.sensors.craterRimDetector.medium())
        telemetry.addData("far", robot.sensors.craterRimDetector.far())
        telemetry.addData("distance", robot.sensors.craterRimDetector.getDistance())
        telemetry.addLine()
        telemetry.addData("down limit", robot.lift.isFullyDown())
        telemetry.addData("up limit", robot.lift.isFullyUp())
        telemetry.addLine()
        telemetry.addData("wallDistance", robot.sensors.backIntoWallDetector.lastKnownDistance)
        robot.sensors.lineDetector.reds.forEach { telemetry.addData("reds", it) }
        robot.sensors.lineDetector.blues.forEach { telemetry.addData("blues", it) }
        telemetry.addData("blues", robot.sensors.lineDetector.blues)
        telemetry.addData("On line", robot.sensors.lineDetector.onLine)
        telemetry.addLine()
        telemetry.addData("extensionTicks", robot.intake.extensionTicks())
        telemetry.addData("extensionInches", robot.intake.extensionInches())
    }
}
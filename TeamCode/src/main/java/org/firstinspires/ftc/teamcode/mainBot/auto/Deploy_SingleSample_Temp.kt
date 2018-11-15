package org.firstinspires.ftc.teamcode.mainBot.auto

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions

@Autonomous
@Config
class Deploy_SingleSample_Temp : Deploy() {
    companion object {
        @JvmField
        var startDistance = 500.0
        @JvmField
        var sideDistance = 0.0
        @JvmField
        var leftAngle = 45.0
        @JvmField
        var rightAngle = -45.0
    }

    override fun run() {
        val sampleAngle: Double = when (robot.vision.tfLite.lastKnownSampleOrder) {
            SampleRandomizedPositions.LEFT -> 45.0
            SampleRandomizedPositions.CENTER, SampleRandomizedPositions.UNKNOWN -> 0.0
            SampleRandomizedPositions.RIGHT -> -45.0
        }
        //super.run()

        robot.drive.resetEncoders()
        robot.drive.startFollowingAngle_setConstants(DriveTerrain.AngleFollowSpeeds.SLOW, 0.0)
        waitWhile { robot.drive.leftTicks() + robot.drive.rightTicks() < startDistance }
        robot.drive.stop()

        sleepSeconds(2.0)

        robot.drive.pidTurn(sampleAngle)

        /*sleepSeconds(2.0)

        when(robot.vision.tfLite.lastKnownSampleOrder){
            SampleRandomizedPositions.LEFT, SampleRandomizedPositions.RIGHT -> {
                robot.drive.startFollowingAngle_setConstants(DriveTerrain.AngleFollowSpeeds.SLOW, sampleAngle)
                waitWhile { robot.drive.leftTicks() + robot.drive.rightTicks() < sideDistance }
            }
            SampleRandomizedPositions.CENTER, SampleRandomizedPositions.UNKNOWN -> {
                robot.drive.openLoopArcade(x = 0.3)
                sleepSeconds(3.0)
            }
        }
        robot.drive.stop()*/
    }
}
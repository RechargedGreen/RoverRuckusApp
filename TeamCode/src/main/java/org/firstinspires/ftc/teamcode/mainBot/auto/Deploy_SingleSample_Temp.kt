package org.firstinspires.ftc.teamcode.mainBot.auto

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions

@Autonomous
@Config
open class Deploy_SingleSample_Temp : Deploy() {
    companion object {
        @JvmField
        var startDistance = 300
        @JvmField
        var sideDistance = 2000
        @JvmField
        var sampleOffSet = 40.0
        @JvmField
        var centerDistance = 1400
        @JvmField
        var parkPower = 0.15
    }

    var ORDER = SampleRandomizedPositions.UNKNOWN

    var park = false

    override fun run() {
        val sampleAngle: Double = when (robot.vision.tfLite.lastKnownSampleOrder) {
            SampleRandomizedPositions.LEFT -> sampleOffSet
            SampleRandomizedPositions.CENTER, SampleRandomizedPositions.UNKNOWN -> 0.0
            SampleRandomizedPositions.RIGHT -> -sampleOffSet
        }
        super.run()
        ORDER = robot.vision.tfLite.lastKnownSampleOrder

        // line up
        robot.drive.deadReckonPID(startDistance, 0.0, DriveTerrain.AngleFollowSpeeds.SLOW)

        sleepSeconds(1.0)

        robot.drive.pidTurn(sampleAngle)

        sleepSeconds(1.0)

        // knock off and park
        when(ORDER){
            SampleRandomizedPositions.LEFT, SampleRandomizedPositions.RIGHT -> {
                robot.drive.deadReckonPID(sideDistance, sampleAngle, DriveTerrain.AngleFollowSpeeds.SLOW)
                sleepSeconds(1.0)
                if(park) {
                    robot.drive.pidTurn(0.0)
                    sleepSeconds(1.0)
                    robot.drive.runTime(parkPower, 2.0)
                }else {
                    robot.drive.deadReckonPID(-sideDistance, sampleAngle, DriveTerrain.AngleFollowSpeeds.SLOW)
                }
            }
            SampleRandomizedPositions.CENTER, SampleRandomizedPositions.UNKNOWN -> {
                robot.drive.deadReckonPID(centerDistance, sampleAngle, DriveTerrain.AngleFollowSpeeds.SLOW)
                sleepSeconds(1.0)
                if(park)
                    robot.drive.runTime(parkPower, 2.0)
                else
                    robot.drive.deadReckonPID(-centerDistance, sampleAngle, DriveTerrain.AngleFollowSpeeds.SLOW)
            }
        }
    }
}
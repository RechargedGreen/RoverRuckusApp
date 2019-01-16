package org.firstinspires.ftc.teamcode.mainBot.auto

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions

@Config
@Autonomous(group = OpModeGroups.PROTOTYPE_AUTO)
class SilverSampleTesting : RR2Auto(StartingPositions.SILVER_HANG){
    companion object {
        @JvmField var leftOffset = 40.0
        @JvmField var rightOffset = 30.0

        @JvmField var rightTicks = 2000
        @JvmField var leftTicks = 2000

        @JvmField var centerDistance = 2000
        @JvmField var lastAngle = CompassDirection.SOUTH_WEST.degrees
    }
    override fun postDeploy() {
        val degree = StartingPositions.SILVER_HANG.angle
        when (ORDER){
            SampleRandomizedPositions.CENTER, SampleRandomizedPositions.UNKNOWN -> {
                robot.drive.deadReckonPID(centerDistance, degree, DriveTerrain.AngleFollowSpeeds.SLOW)
                sleepSeconds(0.1)
                robot.drive.strafeAroundLeft(lastAngle)
            }
            SampleRandomizedPositions.LEFT -> {
                robot.drive.strafeAroundLeft(degree + leftOffset)
                robot.drive.deadReckonPID(leftTicks, degree + leftOffset, DriveTerrain.AngleFollowSpeeds.PARK)
                robot.drive.strafeAroundRight(degree)
                robot.drive.strafeAroundLeft(lastAngle)
            }
            SampleRandomizedPositions.RIGHT -> {
                robot.drive.strafeAroundRight(degree - rightOffset)
                robot.drive.deadReckonPID(rightTicks, degree - rightOffset)
                robot.drive.strafeAroundLeft(lastAngle)
            }

        }
    }
}
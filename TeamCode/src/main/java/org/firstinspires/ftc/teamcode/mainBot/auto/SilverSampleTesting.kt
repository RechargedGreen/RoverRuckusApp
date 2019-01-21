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
        @JvmField var leftOffset = 30.0
        @JvmField var rightOffset = 30.0

        @JvmField var rightTicks = 2000
        @JvmField var rightBackTicks = 500
        @JvmField var leftTicks = 2000

        @JvmField var centerDistance = 2000
        @JvmField var centerBackDistance = 100
        @JvmField var lastAngle = CompassDirection.SOUTH_WEST.degrees

        @JvmField var leftPostDistance = 0
        @JvmField var centerPostDistance = 500
        @JvmField var rightPostDistance = 1000
    }
    override fun postDeploy() {
        val degree = StartingPositions.SILVER_HANG.angle
        when (ORDER){
            SampleRandomizedPositions.CENTER, SampleRandomizedPositions.UNKNOWN -> {
                robot.drive.deadReckonPID(centerDistance, degree, DriveTerrain.AngleFollowSpeeds.SLOW)
                sleepSeconds(0.5)
                robot.drive.deadReckonPID(-centerBackDistance, degree, DriveTerrain.AngleFollowSpeeds.SLOW)
                robot.drive.strafeAroundLeft(lastAngle)
            }
            SampleRandomizedPositions.LEFT -> {
                robot.drive.strafeAroundLeft(degree + leftOffset)
                robot.drive.deadReckonPID(leftTicks, degree + leftOffset, DriveTerrain.AngleFollowSpeeds.FAST)
                robot.drive.strafeAroundRight(degree)
                robot.drive.strafeAroundLeft(lastAngle)
            }
            SampleRandomizedPositions.RIGHT -> {
                robot.drive.strafeAroundRight(degree - rightOffset)
                robot.drive.deadReckonPID(rightTicks, degree - rightOffset, DriveTerrain.AngleFollowSpeeds.SLOW)
                robot.drive.deadReckonPID(-rightBackTicks, degree - rightOffset, DriveTerrain.AngleFollowSpeeds.SLOW)
                robot.drive.strafeAroundLeft(lastAngle)
            }
        }

        robot.drive.deadReckonPID(-when(ORDER){
            SampleRandomizedPositions.LEFT -> leftPostDistance
            SampleRandomizedPositions.CENTER, SampleRandomizedPositions.UNKNOWN -> centerPostDistance
            SampleRandomizedPositions.RIGHT -> rightPostDistance
        }, lastAngle, DriveTerrain.AngleFollowSpeeds.FAST)
    }
}
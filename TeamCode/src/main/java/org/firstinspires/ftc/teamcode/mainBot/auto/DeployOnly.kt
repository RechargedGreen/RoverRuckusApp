package org.firstinspires.ftc.teamcode.mainBot.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@Autonomous(group = OpModeGroups.AUTO)
class DeployOnly : RR2Auto(StartingPositions.ANY_HANG) {
    override fun postDeploy() = robot.drive.deadReckonPID(100, 0.0, DriveTerrain.AngleFollowSpeeds.SLOW)
}
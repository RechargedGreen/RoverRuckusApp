package org.firstinspires.ftc.teamcode.mainBot.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@Autonomous(group = OpModeGroups.AUTO)
class GoldDeployFastSampleTeamMarker : RR2Auto(StartingPositions.GOLD_HANG){
    override fun postDeploy() = sample(SampleCollectionType.LANDER_DRIVE_FAST_TEAM_MARKER           )
}
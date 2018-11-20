package org.firstinspires.ftc.teamcode.mainBot.auto

/**
 * Created by David Lukens on 11/20/2018.
 */
class DeployFastSampleTeamMarker : RR2Auto(StartingPositions.GOLD_HANG){
    override fun postDeploy() = sample(SampleCollectionType.LANDER_DRIVE_FAST_TEAM_MARKER)
}
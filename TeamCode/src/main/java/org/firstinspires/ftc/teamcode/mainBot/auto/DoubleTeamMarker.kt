package org.firstinspires.ftc.teamcode.mainBot.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@Autonomous(group = OpModeGroups.MAIN_AUTO)
class DoubleTeamMarker : RR2Auto(StartingPositions.SILVER_HANG){
    override fun postDeploy() {
        sample(SampleCollectionType.LANDER_EXTENSION_SILVER)
        silverSampleWallLinup()
        intoDepotSilver()
        teamMarker(true)
        sample(SampleCollectionType.DEPOT_EXTENSION)
        robot.drive.pidTurn(CompassDirection.SOUTH.degrees + 10)
        robot.drive.deadReckonPID(1000, CompassDirection.SOUTH.degrees + 10, DriveTerrain.AngleFollowSpeeds.PARK)
        park(CompassDirection.SOUTH.degrees)
    }
}
package org.firstinspires.ftc.teamcode.mainBot.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@Autonomous(group = OpModeGroups.MAIN_AUTO)
class SilverFull : RR2Auto(StartingPositions.SILVER_HANG){
    override fun postDeploy() {
        sample(SampleCollectionType.LANDER_EXTENSION_SILVER)
        silverSampleWallLinup()
        intoDepotSilver()
        teamMarker(true)
        robot.drive.pidTurn(CompassDirection.SOUTH.degrees + intoWallOffset)// not parking issue was with the turn not completing
        robot.drive.deadReckonPID(1000, CompassDirection.SOUTH.degrees + intoWallOffset, DriveTerrain.AngleFollowSpeeds.SLOW)
        park(CompassDirection.SOUTH.degrees)
    }
}
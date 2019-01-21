package org.firstinspires.ftc.teamcode.mainBot.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@Autonomous(group = OpModeGroups.MAIN_AUTO)
class DoubleSample : RR2Auto(StartingPositions.SILVER_HANG){
    override fun postDeploy() {
        sample(SampleCollectionType.DRIVE_SILVER)
        silverSampleWallLinup()
        intoDepotSilver()
        teamMarker(true)
        sample(SampleCollectionType.DEPOT_EXTENSION)
        /*robot.drive.pidTurn(CompassDirection.SOUTH.degrees + intoWallOffset)
        robot.drive.deadReckonPID(1000, CompassDirection.SOUTH.degrees + intoWallOffset, DriveTerrain.AngleFollowSpeeds.SLOW)*/
        park(CompassDirection.SOUTH.degrees)
    }
}
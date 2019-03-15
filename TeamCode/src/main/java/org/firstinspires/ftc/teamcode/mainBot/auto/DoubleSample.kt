package org.firstinspires.ftc.teamcode.mainBot.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@Autonomous(group = OpModeGroups.MAIN_AUTO)
class DoubleSample : RR2Auto(StartingPositions.SILVER_HANG) {
    @Throws(InterruptedException::class)
    override fun postDeploy() {
        sample(SampleCollectionType.DRIVE_SILVER)
        silverSampleWallLinup()
        intoDepotSilver()
        teamMarker(true)
        sample(SampleCollectionType.DRIVE_DEPOT)
        /*robot.drive.pidTurn(CompassDirection.SOUTH.degrees + intoWallOffset)
        robot.drive.deadReckonPID(1000, CompassDirection.SOUTH.degrees + intoWallOffset, DriveTerrain.AngleFollowSpeeds.SLOW)*/
        robot.drive.pidTurn(CompassDirection.SOUTH.degrees + 5.0)
        robot.drive.deadReckonPID(1000, CompassDirection.SOUTH.degrees + 5.0, DriveTerrain.AngleFollowSpeeds.FAST)
        //park(CompassDirection.SOUTH.degrees, false)
    }
}
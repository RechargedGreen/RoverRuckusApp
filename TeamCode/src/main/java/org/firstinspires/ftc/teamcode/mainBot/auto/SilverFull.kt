package org.firstinspires.ftc.teamcode.mainBot.auto

import com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain.DiffDrive
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain
import org.firstinspires.ftc.teamcode.mainBot.hardware.Lift
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@Autonomous(group = OpModeGroups.MAIN_AUTO)
class SilverFull : RR2Auto(StartingPositions.SILVER_HANG){
    override fun postDeploy() {
        sample(SampleCollectionType.LANDER_EXTENSION_SILVER)
        silverSampleWallLinup()
        robot.drive.startFollowingAngle_setConstants(DriveTerrain.AngleFollowSpeeds.PARK, CompassDirection.SOUTH.degrees, true, DiffDrive.AnglePIDType.STRAIGHT)
        robot.lift.state = Lift.State.UP
        robot.sensors.lineDetector.reset()
        waitTill { robot.sensors.lineDetector.hasHit }
        robot.drive.stop()
        teamMarker(true)
        park(CompassDirection.SOUTH.degrees)
    }
}
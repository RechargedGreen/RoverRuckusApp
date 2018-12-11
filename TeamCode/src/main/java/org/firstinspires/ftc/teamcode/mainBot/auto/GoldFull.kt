package org.firstinspires.ftc.teamcode.mainBot.auto

import com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain.DiffDrive
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain
import org.firstinspires.ftc.teamcode.mainBot.hardware.Intake
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups
import kotlin.math.absoluteValue

@Autonomous(group = OpModeGroups.MAIN_AUTO)
class GoldFull : RR2Auto(StartingPositions.GOLD_HANG){
    override fun postDeploy() {
        sample(SampleCollectionType.LANDER_DRIVE_FAST_TEAM_MARKER)
        robot.intake.intakeState = Intake.IntakeState.STOP
        robot.drive.pidTurn(CompassDirection.WEST.degrees)
        prepCraterSense()
        robot.drive.startFollowingAngle_setConstants(DriveTerrain.AngleFollowSpeeds.PARK, CompassDirection.WEST.degrees, false, DiffDrive.AnglePIDType.STRAIGHT)
        waitTill { hittingCrater() }
        robot.drive.stop()
    }
}
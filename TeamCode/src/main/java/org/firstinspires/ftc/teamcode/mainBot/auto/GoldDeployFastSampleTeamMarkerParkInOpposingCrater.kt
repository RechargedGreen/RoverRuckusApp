package org.firstinspires.ftc.teamcode.mainBot.auto

import com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain.DiffDrive
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain
import org.firstinspires.ftc.teamcode.mainBot.hardware.Intake
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups
import kotlin.math.absoluteValue

@Autonomous(group = OpModeGroups.AUTO)
class GoldDeployFastSampleTeamMarkerParkInOpposingCrater : RR2Auto(StartingPositions.GOLD_HANG){
    override fun postDeploy() {
        sample(SampleCollectionType.LANDER_DRIVE_FAST_TEAM_MARKER)
        robot.intake.intakeState = Intake.IntakeState.STOP
        robot.drive.pidTurn(CompassDirection.EAST.degrees)
        robot.drive.imu.resetX()
        robot.drive.imu.resetY()
        robot.drive.startFollowingAngle_setConstants(DriveTerrain.AngleFollowSpeeds.PARK, CompassDirection.EAST.degrees, true, DiffDrive.AnglePIDType.STRAIGHT)
        waitTill { robot.drive.imu.getY().absoluteValue + robot.drive.imu.getX().absoluteValue > 7 }
    }
}
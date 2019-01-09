package org.firstinspires.ftc.teamcode.mainBot.diagnostics

import com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain.DiffDrive
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.auto.RR2Auto
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain

@Autonomous
class LineDetectTest : RR2Auto(StartingPositions.ANY_HANG, 0.0){
    override fun postDeploy() {
        robot.sensors.lineDetector.reset()
        robot.drive.startFollowingAngle_setConstants(DriveTerrain.AngleFollowSpeeds.LINE_DETECT, 0.0, true, DiffDrive.AnglePIDType.STRAIGHT)
        waitTill { robot.sensors.lineDetector.hasHit }
        robot.drive.stop()
    }
}
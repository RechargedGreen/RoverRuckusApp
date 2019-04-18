package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.auto.RR2Auto

@Autonomous
class PIDDriveTest : RR2Auto(StartingPositions.ANY_HANG){
    override fun postDeploy() {
        robot.drive.followSource(PIDDriveSource(48.0, 0.0, 1.0, 1.0, robot))
        sleepSeconds(2.0)
        robot.drive.followSource(PIDDriveSource(-48.0, 0.0, 1.0, 1.0, robot))
        sleepSeconds(2.0)

        robot.drive.followSource(PIDDriveSource(96.0, 0.0, 1.0, 1.0, robot))
        sleepSeconds(2.0)
        robot.drive.followSource(PIDDriveSource(-96.0, 0.0, 1.0, 1.0, robot))
        sleepSeconds(2.0)
    }
}
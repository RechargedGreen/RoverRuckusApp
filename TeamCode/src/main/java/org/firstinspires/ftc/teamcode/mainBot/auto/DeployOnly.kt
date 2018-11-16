package org.firstinspires.ftc.teamcode.mainBot.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@Autonomous(group = OpModeGroups.AUTO)
class DeployOnly : Deploy() {
    override fun run(){
        super.run()
        robot.drive.deadReckonPID(500, 0.0, DriveTerrain.AngleFollowSpeeds.SLOW)
    }
}
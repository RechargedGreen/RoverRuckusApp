package org.firstinspires.ftc.teamcode.mainBot.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@Autonomous(group = OpModeGroups.BACKUP_AUTO)
class DeployAndPark : RR2Auto(StartingPositions.SILVER_HANG) {
    override fun postDeploy() = park(StartingPositions.SILVER_HANG.angle)
}
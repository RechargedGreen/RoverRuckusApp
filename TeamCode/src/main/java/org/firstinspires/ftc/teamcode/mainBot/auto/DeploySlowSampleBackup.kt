package org.firstinspires.ftc.teamcode.mainBot.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@Autonomous(group = OpModeGroups.BACKUP_AUTO)
open class DeploySlowSampleBackup : RR2Auto(StartingPositions.ANY_HANG) {
    override fun postDeploy() = sample(SampleCollectionType.LANDER_DRIVE_SLOW_BACKUP)
}
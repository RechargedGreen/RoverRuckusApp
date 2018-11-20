package org.firstinspires.ftc.teamcode.mainBot.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous
open class DeploySlowSampleBackup : RR2Auto(StartingPositions.ANY_HANG) {
    override fun postDeploy() = sample(SampleCollectionType.LANDER_DRIVE_SLOW_BACKUP)
}
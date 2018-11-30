package org.firstinspires.ftc.teamcode.mainBot.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

/**
 * Created by David Lukens on 11/15/2018.
 */
@Autonomous(group = OpModeGroups.BACKUP_AUTO)
class DeploySlowSamplePark : RR2Auto(StartingPositions.SILVER_HANG){
    override fun postDeploy() = sample(SampleCollectionType.LANDER_DRIVE_SLOW_PARK)
}
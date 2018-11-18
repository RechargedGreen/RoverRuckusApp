package org.firstinspires.ftc.teamcode.mainBot.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

/**
 * Created by David Lukens on 11/15/2018.
 */
@Autonomous
class Deploy_SingleSample_Park_Temp : RR2Auto(StartingPositions.SILVER_HANG){
    override fun postDeploy() = sample(SampleCollectionType.LANDER_DRIVE_PARK)
}
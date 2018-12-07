package org.firstinspires.ftc.teamcode.mainBot.auto

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(group = OpModeGroups.MAIN_AUTO)
class SilverFull : RR2Auto(StartingPositions.SILVER_HANG){
    override fun postDeploy() {
        sample(SampleCollectionType.LANDER_EXTENSION_SILVER)
        silverSampleWallLinup()
    }
}
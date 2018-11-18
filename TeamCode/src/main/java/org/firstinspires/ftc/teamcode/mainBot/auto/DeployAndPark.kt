package org.firstinspires.ftc.teamcode.mainBot.auto

import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@Autonomous(group = OpModeGroups.AUTO)
class DeployAndPark : RR2Auto(StartingPositions.SILVER_HANG){
    override fun postDeploy() {
        robot.drive.openLoopArcade(x = 0.15)
        sleepSeconds(seconds = 5.0)
        robot.drive.stop()
    }
}
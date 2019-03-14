package org.firstinspires.ftc.teamcode.mainBot.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@Autonomous(group = OpModeGroups.MAIN_AUTO)
class GoldFull : RR2Auto(StartingPositions.GOLD_HANG) {
    override fun postDeploy() {
        SuperFastGoldSample.doStuff(this)
        //sample(SampleCollectionType.LANDER_DRIVE_FAST_TEAM_MARKER)
        robot.drive.pidTurn(CompassDirection.WEST.degrees + 15)
        park(CompassDirection.WEST.degrees)
    }
}
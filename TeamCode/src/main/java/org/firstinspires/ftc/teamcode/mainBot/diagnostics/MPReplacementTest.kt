package org.firstinspires.ftc.teamcode.mainBot.diagnostics

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.auto.RR2Auto
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

/**
 * Created by David Lukens on 1/15/2019.
 */
@TeleOp(group = OpModeGroups.TELE_DIAGNOSTICS)
class MPReplacementTest : RR2Auto(StartingPositions.ANY_HANG) {
    @Throws(InterruptedException::class)
    override fun postDeploy() {
        loop {
            robot.drive.drive(48.0, 0.0)
            sleepSeconds(5.0)
            robot.drive.drive(-48.0, 0.0)
            sleepSeconds(5.0)
            robot.drive.drive(96.0, 0.0)
            sleepSeconds(5.0)
            robot.drive.drive(-96.0, 0.0)
            sleepSeconds(5.0)
        }
    }
}
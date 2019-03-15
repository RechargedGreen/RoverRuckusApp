package org.firstinspires.ftc.teamcode.mainBot.teleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

/**
 * Created by David Lukens on 9/27/2018.
 */

@TeleOp(name = Competition.NAME, group = OpModeGroups.TELEOP)
class Competition : PracticeJVoExtension() {
    @Throws(InterruptedException::class)
    override fun onStart() {
        practice = false
        super.onStart()
    }

    companion object {
        const val NAME = "Competition"
    }
}
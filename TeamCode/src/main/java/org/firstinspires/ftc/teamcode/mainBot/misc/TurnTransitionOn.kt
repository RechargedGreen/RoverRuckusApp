package org.firstinspires.ftc.teamcode.mainBot.misc

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.hardware.Static

@TeleOp(group = OpModeGroups.TELE_MISC)
class TurnTransitionOn : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        Static.transitionOnAutoEnd = true
    }
}
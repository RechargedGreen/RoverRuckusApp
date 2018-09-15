package org.firstinspires.ftc.teamcode.prototypes

import com.david.rechargedkotlinlibrary.internal.opMode.CompetetionTele
import com.david.rechargedkotlinlibrary.internal.opMode.PracticeTeleOp
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

/**
 * Created by David Lukens on 9/10/2018.
 */
@TeleOp(name = RR1Tele.NAME)
class RR1Tele : PracticeTeleOp<RR1Bot>({ opmode -> RR1Bot(opmode) }) {
    companion object {
        const val NAME = "RR1Tele"
    }
    override fun onLoop() = robot.drive.openLoopPowerWheels(l = c1.ly, r = c1.ry)
}
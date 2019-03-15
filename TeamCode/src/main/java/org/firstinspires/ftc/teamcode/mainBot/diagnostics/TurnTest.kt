package org.firstinspires.ftc.teamcode.mainBot.diagnostics

import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(group = OpModeGroups.TELE_DIAGNOSTICS)
class TurnTest : FluidAuto<HardwareClass>({ opMode -> HardwareClass(opMode) }) {
    @Throws(InterruptedException::class)
    override fun run() {
        robot.drive.pidTurn(90.0)
        sleepSeconds(1.0)
        robot.drive.pidTurn(0.0)
        sleepSeconds(1.0)
        robot.drive.strafeAroundLeft(90.0)
        sleepSeconds(1.0)
        robot.drive.strafeAroundLeft(0.0)
        sleepSeconds(1.0)
        robot.drive.strafeAroundRight(-90.0)
        sleepSeconds(1.0)
        robot.drive.strafeAroundRight(0.0)
        sleepSeconds(0.0)
    }
}
package org.firstinspires.ftc.teamcode.mainBot.misc

import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass

@TeleOp(group = OpModeGroups.TELE_DIAGNOSTICS)
class BoxChecker : FluidAuto<HardwareClass>({ opMode -> HardwareClass(opMode) }) {
    @Throws(InterruptedException::class)
    override fun run() {
    }
}
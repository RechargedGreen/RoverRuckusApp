package org.firstinspires.ftc.teamcode.vision

import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(group = OpModeGroups.TELE_MISC)
class VisionTest : FluidAuto<HardwareClass>({ opMode -> HardwareClass(opMode) }) {
    override fun tillStart() {
        telemetry.addData("Order", robot.vision.tfLite.lastKnownSampleOrder)
        telemetry.update()
    }

    override fun run() {
    }
}
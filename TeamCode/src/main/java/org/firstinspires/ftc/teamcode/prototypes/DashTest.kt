package org.firstinspires.ftc.teamcode.prototypes

import com.david.rechargedkotlinlibrary.internal.opMode.PracticeTeleOp
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(group = OpModeGroups.TELE_MISC)
class DashTest : PracticeTeleOp<HardwareClass>({ opMode -> HardwareClass(opMode) }) {
    @Throws(InterruptedException::class)
    override fun onLoop() {
        packet.put("lt", c1.lt)
        packet.put("rt", c1.rt)
        packet.put("lx", c1.lx)
        packet.put("rx", c1.rx)
        packet.put("ly", c1.ly)
        packet.put("ry", c1.ry)
    }
}
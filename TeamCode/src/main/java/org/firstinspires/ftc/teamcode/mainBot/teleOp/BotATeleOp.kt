package org.firstinspires.ftc.teamcode.mainBot.teleOp

import com.david.rechargedkotlinlibrary.internal.opMode.PracticeTeleOp
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass

@TeleOp(name = BotATeleOp.NAME)
class BotATeleOp : PracticeTeleOp<HardwareClass>({ opMode-> HardwareClass(opMode) }){
    override fun onLoop() {
        robot.drive.openLoopPowerWheels(c1.ly, c1.ry)
    }
    companion object {
        const val NAME = "TeleOp A"
    }
}
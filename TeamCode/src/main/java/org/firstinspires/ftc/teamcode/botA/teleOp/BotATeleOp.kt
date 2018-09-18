package org.firstinspires.ftc.teamcode.botA.teleOp

import com.david.rechargedkotlinlibrary.internal.opMode.PracticeTeleOp
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.botA.hardware.BotAHardwareClass

@TeleOp
class BotATeleOp : PracticeTeleOp<BotAHardwareClass>({ opMode-> BotAHardwareClass(opMode) }){
    override fun onLoop() {
        robot.drive.openLoopPowerWheels(c1.ly, c1.ry)
    }
}
package org.firstinspires.ftc.teamcode.mainBot.teleOp

import com.david.rechargedkotlinlibrary.internal.opMode.PracticeTeleOp
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.hardware.Lift
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(name = Practice.NAME, group = OpModeGroups.TELEOP)
open class Practice : PracticeTeleOp<HardwareClass>({ opMode -> HardwareClass(opMode) }) {
    override fun onLoop() {
        robot.drive.openLoopPowerWheels(c1.ly, c1.ry)
        robot.lift.setOpenLoopPower(c1.rt - c1.lt)
        //robot.lift.state = if(gamepad1.right_bumper) Lift.State.UP else Lift.State.DOWN
    }

    companion object {
        const val NAME = "Practice"
    }
}
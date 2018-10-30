package org.firstinspires.ftc.teamcode.mainBot.teleOp

import com.david.rechargedkotlinlibrary.internal.opMode.PracticeTeleOp
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.hardware.Lift
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups
import kotlin.math.absoluteValue

@TeleOp(name = Practice.NAME, group = OpModeGroups.TELEOP)
open class Practice : PracticeTeleOp<HardwareClass>({ opMode -> HardwareClass(opMode) }) {
    val deadBand = 0.05
    override fun onLoop() {
        val l = c1.ly * (1.0 - c1.lt)
        val r = c1.ry * (1.0 - c1.rt)
        robot.drive.openLoopPowerWheels(if(l.absoluteValue > deadBand) l else 0.0, if(r.absoluteValue > deadBand) r else 0.0)
        val lift = c2.ly
        robot.lift.setOpenLoopPower(if(lift.absoluteValue > deadBand) lift else 0.0, useFailSafes = false)
        telemetry.addData("lift power", lift)
        //robot.lift.state = if(gamepad1.right_bumper) Lift.State.UP else Lift.State.DOWN
    }

    companion object {
        const val NAME = "Practice"
    }
}
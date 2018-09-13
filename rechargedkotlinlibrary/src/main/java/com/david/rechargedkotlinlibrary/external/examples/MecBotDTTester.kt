package com.david.rechargedkotlinlibrary.external.examples

import com.david.rechargedkotlinlibrary.internal.opMode.PracticeTeleOp
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

/**
 * Created by David Lukens on 8/8/2018.
 */
@TeleOp(name = MecBotDTTester.NAME)
@Disabled
class MecBotDTTester : PracticeTeleOp<MecBot>({ opMode -> MecBot(opMode) }) {
    companion object {
        const val NAME = "MecBotDTTester"
    }

    var lf = false
    var lb = false
    var rf = false
    var rb = false

    override fun onLoop() {
        val speed = c1.ry
        lf = c1.a
        lb = c1.b
        rf = c1.x
        rb = c1.y
        if (c1.rb) {
            if (lf) robot.drive.resetLFEncoder()
            if (lb) robot.drive.resetLBEncoder()
            if (rf) robot.drive.resetRFEncoder()
            if (rb) robot.drive.resetRBEncoder()
        }
        robot.drive.setMotorPowers(if (lf) speed else 0.0, if (lb) speed else 0.0, if (rf) speed else 0.0, if (rb) speed else 0.0)
        telemetry.addData("speed", speed)
        telemetry.addLine("a for lf")
        telemetry.addLine("b for lb")
        telemetry.addLine("x for rf")
        telemetry.addLine("y for rb")
        telemetry.addData("lfPos", robot.drive.lfTicks())
        telemetry.addData("lbPos", robot.drive.lbTicks())
        telemetry.addData("rfPos", robot.drive.rfTicks())
        telemetry.addData("rbPos", robot.drive.rbTicks())
    }
}
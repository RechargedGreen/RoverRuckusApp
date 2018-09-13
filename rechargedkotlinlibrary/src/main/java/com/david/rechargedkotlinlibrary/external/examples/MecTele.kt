package com.david.rechargedkotlinlibrary.external.examples

import com.david.rechargedkotlinlibrary.internal.opMode.CompetetionTele
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

/**
 * Created by David Lukens on 8/2/2018.
 */
@TeleOp(name = MecTele.NAME)
@Disabled
class MecTele : CompetetionTele<MecBot>({ opMode -> MecBot(opMode) }) {
    companion object {
        const val NAME = "MecTele"
    }

    override fun onLoop() {
        robot.drive.powerTranslation((c1.ly + c1.ry) / 2, c1.rt - c1.lt, (c1.ly - c1.ry) / 2)
        telemetry.addData("ly", c1.ly)
        telemetry.addData("ry", c1.ry)
        telemetry.addData("lt", c1.lt)
        telemetry.addData("rt", c1.rt)
        telemetry.addData("wheel positions", robot.drive.getWheelPositions())
        telemetry.addData("Pos", robot.drive.getPos())
    }
}
package org.firstinspires.ftc.teamcode.iterative.testing

import org.firstinspires.ftc.teamcode.iterative.lib.IterativeOpMode

class IterativeTest : IterativeOpMode(false){
    override fun eventLoop() {
        telemetry.addLine("looping")
        telemetry.addLine("press a to stop")
        if(gamepad1.a)
            end()
    }
}
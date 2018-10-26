package org.firstinspires.ftc.teamcode.mainBot.diagnostics

import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass

class TurnMPTester : FluidAuto<HardwareClass>({ opMode -> HardwareClass(opMode) }){
    override fun run() {
        robot.drive.waitOnTrajectory(trajectory = robot.drive.trajectoryBuilder()
                .turnTo(Math.toRadians(180.0))
                .waitFor(2.0)
                .turnTo(Math.toRadians(90.0))
                .waitFor(2.0)
                .turnTo(Math.toRadians(0.0))
                .waitFor(2.0)
                .turn(Math.toRadians(360.0))
                .build()
        )
    }
}
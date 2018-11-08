package org.firstinspires.ftc.teamcode.mainBot.diagnostics

import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(group = OpModeGroups.TELE_DIAGNOSTICS)
class StraightLineMPTester : FluidAuto<HardwareClass>({ opMode -> HardwareClass(opMode) }){
    override fun run() {
        robot.drive.waitOnTrajectory(trajectory = robot.drive.trajectoryBuilder()
                .forward(48.0)
                .waitFor(5.0)
                .back(48.0)
                .build()
        )
    }
}
package org.firstinspires.ftc.teamcode.mainBot.auto

import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.hardware.SuperSystem
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@Autonomous(name = "ParkOnly", group = OpModeGroups.FLUID_AUTO)
class ParkOnlyAuto : FluidAuto<HardwareClass>({ opMode -> HardwareClass(opMode) }) {
    override fun run() {
        robot.drive.openLoopPowerWheels(0.5, 0.5)
        sleep(3000)
        robot.drive.openLoopPowerWheels(0.0, 0.0)
        robot.superSystem.setState(SuperSystem.State.RESET)
    }
}
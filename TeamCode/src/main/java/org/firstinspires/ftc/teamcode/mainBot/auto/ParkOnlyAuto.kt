package org.firstinspires.ftc.teamcode.mainBot.auto

import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.hardware.SuperSystem

class ParkOnlyAuto : FluidAuto<HardwareClass>({ opMode-> HardwareClass(opMode) }){
    override fun run() {
        robot.drive.openLoopPowerWheels(0.5, 0.5)
        sleep(3000)
        robot.drive.openLoopPowerWheels(0.0, 0.0)
        robot.superSystem.setState(SuperSystem.State.RESET)
    }
}
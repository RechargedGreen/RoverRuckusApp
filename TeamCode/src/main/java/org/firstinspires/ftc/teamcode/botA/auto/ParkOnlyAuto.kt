package org.firstinspires.ftc.teamcode.botA.auto

import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import org.firstinspires.ftc.teamcode.botA.hardware.BotAHardwareClass

class ParkOnlyAuto : FluidAuto<BotAHardwareClass>({ opMode-> BotAHardwareClass(opMode) }){
    override fun run() {
        robot.drive.openLoopPowerWheels(0.5, 0.5)
        sleep(3000)
        robot.drive.openLoopPowerWheels(0.0, 0.0)
    }
}
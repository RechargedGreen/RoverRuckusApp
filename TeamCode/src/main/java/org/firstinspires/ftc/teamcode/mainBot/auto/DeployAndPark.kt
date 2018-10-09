package org.firstinspires.ftc.teamcode.mainBot.auto

import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass

class DeployAndPark : FluidAuto<HardwareClass>({ opMode -> HardwareClass(opMode) }){
    override fun run() {
        robot.lift.deploy()
        robot.drive.startFollowingAngle_setConstants(angle = 0.0)
        sleep(3000)
        robot.drive.stop()
    }
}
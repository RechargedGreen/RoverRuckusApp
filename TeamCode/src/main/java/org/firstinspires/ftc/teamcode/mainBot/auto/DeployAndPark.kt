package org.firstinspires.ftc.teamcode.mainBot.auto

import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass

class DeployAndPark : FluidAuto<HardwareClass>({ opMode -> HardwareClass(opMode) }){
    override fun run() {
        robot.lift.deploy()

        robot.drive.resetEncoders()
        robot.drive.startFollowingAngle_setConstants(angle = 0.0)
        waitWhile { robot.drive.leftTicks() + robot.drive.rightTicks() < 5000 }

        robot.drive.stop()
    }
}
package org.firstinspires.ftc.teamcode.mainBot.auto

import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

/**
 * Created by David Lukens on 11/2/2018.
 */
@Autonomous(group = OpModeGroups.AUTO)
class Deploy :FluidAuto<HardwareClass>({ opMode ->  HardwareClass(opMode) }){
    override fun run() = robot.lift.deploy()
}
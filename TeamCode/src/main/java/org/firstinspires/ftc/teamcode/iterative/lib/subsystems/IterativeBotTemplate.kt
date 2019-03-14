package org.firstinspires.ftc.teamcode.iterative.lib.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * Created by David Lukens on 2/15/2019.
 */
interface IterativeBotTemplate {
    fun initHardware(hMap: HardwareMap, autonomous: Boolean, subsystemManager: SubsystemManager)
}
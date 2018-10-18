package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem

class SuperSystem(val robot: HardwareClass) : MTSubsystem {
    enum class State {
        RESET
    }

    fun setState(state: State) {
        when (state) {
            State.RESET -> {
                robot.intake.state = Intake.State.STOPPED
                robot.dumper.dumpState = Dumper.DumpState.HOLD
            }
        }
    }

    override fun update() {}
    override fun start() {}
}
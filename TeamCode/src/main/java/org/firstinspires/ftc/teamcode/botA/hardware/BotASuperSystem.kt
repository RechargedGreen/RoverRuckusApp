package org.firstinspires.ftc.teamcode.botA.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem

class BotASuperSystem(val robot:BotAHardwareClass) : MTSubsystem {
    enum class State{
        RESET
    }

    fun setState(state:State){
        when(state){
            State.RESET -> {
                robot.intake.state = Intake.State.STOPPED
                robot.dumper.flipState = Dumper.FlipState.FRONT
                robot.dumper.dumpState = Dumper.DumpState.HOLD
            }
        }
    }

    override fun update() {}
    override fun start() {}
}
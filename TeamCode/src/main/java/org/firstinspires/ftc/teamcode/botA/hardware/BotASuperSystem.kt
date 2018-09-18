package org.firstinspires.ftc.teamcode.botA.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem

class BotASuperSystem(val robot:BotAHardwareClass) : MTSubsystem {
    enum class State{
        RESET
    }

    fun setState(state:State){
    }

    override fun update() {}
    override fun start() {}
}
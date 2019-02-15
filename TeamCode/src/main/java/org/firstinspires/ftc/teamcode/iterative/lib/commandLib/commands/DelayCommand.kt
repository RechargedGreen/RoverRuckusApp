package org.firstinspires.ftc.teamcode.iterative.lib.commandLib.commands

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.iterative.lib.commandLib.Command

class DelayCommand(private val seconds: Double) : Command {
    private val timer = ElapsedTime()
    override fun start() = timer.reset()
    override fun periodic() {}
    override fun isComplete(): Boolean = timer.seconds() >= seconds
    override fun end() {}
}
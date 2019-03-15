package org.firstinspires.ftc.teamcode.iterative.lib.commandLib.commands

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.iterative.lib.commandLib.Command

class DelayCommand(private val seconds: Double) : Command {
    private val timer = ElapsedTime()
    @Throws(InterruptedException::class)
    override fun start() = timer.reset()
    @Throws(InterruptedException::class)
    override fun periodic() {}
    @Throws(InterruptedException::class)
    override fun isComplete(): Boolean = timer.seconds() >= seconds
    @Throws(InterruptedException::class)
    override fun end() {}
}
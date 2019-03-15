package org.firstinspires.ftc.teamcode.iterative.lib.commandLib.commands

import org.firstinspires.ftc.teamcode.iterative.lib.commandLib.Command

class Run(private val task: () -> Unit) : Command {
    @Throws(InterruptedException::class)
    override fun start() = task()
    @Throws(InterruptedException::class)
    override fun periodic() {}
    @Throws(InterruptedException::class)
    override fun isComplete(): Boolean = true
    @Throws(InterruptedException::class)
    override fun end() {}
}
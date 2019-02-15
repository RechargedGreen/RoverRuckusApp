package org.firstinspires.ftc.teamcode.iterative.lib.commandLib.commands

import org.firstinspires.ftc.teamcode.iterative.lib.commandLib.Command

class Run(private val task: () -> Unit) : Command {
    override fun start() = task()
    override fun periodic() {}
    override fun isComplete(): Boolean = true
    override fun end() {}
}
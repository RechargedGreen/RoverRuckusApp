package org.firstinspires.ftc.teamcode.iterative.lib.commandLib.commands

import org.firstinspires.ftc.teamcode.iterative.lib.commandLib.Command

class WaitTill(private val condition: () -> Boolean, private val periodicFunc: () -> Unit = {}) : Command {
    override fun start() {}
    override fun periodic() = periodicFunc()
    override fun isComplete(): Boolean = condition()
    override fun end() {}
}
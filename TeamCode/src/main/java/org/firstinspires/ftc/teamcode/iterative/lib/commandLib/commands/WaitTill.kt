package org.firstinspires.ftc.teamcode.iterative.lib.commandLib.commands

import org.firstinspires.ftc.teamcode.iterative.lib.commandLib.Command

class WaitTill(private val condition: () -> Boolean, private val periodicFunc: () -> Unit = {}) : Command {
    @Throws(InterruptedException::class)
    override fun start() {
    }

    @Throws(InterruptedException::class)
    override fun periodic() = periodicFunc()

    @Throws(InterruptedException::class)
    override fun isComplete(): Boolean = condition()

    @Throws(InterruptedException::class)
    override fun end() {
    }
}
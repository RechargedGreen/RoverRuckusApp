package org.firstinspires.ftc.teamcode.iterative.lib.commandLib

interface Command {
    @Throws(InterruptedException::class)
    fun start()

    @Throws(InterruptedException::class)
    fun periodic()

    @Throws(InterruptedException::class)
    fun isComplete(): Boolean

    @Throws(InterruptedException::class)
    fun end()
}
package org.firstinspires.ftc.teamcode.iterative.lib.commandLib

interface CommandScheduler {
    @Throws(InterruptedException::class)
    fun run(command: Command)

    @Throws(InterruptedException::class)
    fun periodic()
}
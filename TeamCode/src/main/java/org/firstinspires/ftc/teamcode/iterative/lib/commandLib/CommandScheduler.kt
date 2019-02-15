package org.firstinspires.ftc.teamcode.iterative.lib.commandLib

interface CommandScheduler {
    fun run(command: Command)
    fun periodic()
}
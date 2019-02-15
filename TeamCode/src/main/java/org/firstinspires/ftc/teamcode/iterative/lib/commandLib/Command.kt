package org.firstinspires.ftc.teamcode.iterative.lib.commandLib

interface Command {
    fun start()
    fun periodic()
    fun isComplete():Boolean
    fun stop()
}
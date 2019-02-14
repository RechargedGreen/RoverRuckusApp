package org.firstinspires.ftc.teamcode.iterative.lib.commandLib

interface Command {
    fun start()
    fun loop()
    fun condition():Boolean
    fun stop()
}
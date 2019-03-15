package org.firstinspires.ftc.teamcode.iterative.lib.subsystems

interface Updatable {
    @Throws(InterruptedException::class)
    fun update()
}
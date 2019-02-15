package org.firstinspires.ftc.teamcode.iterative.lib.subsystems

class SubsystemManager {
    private val updatables = ArrayList<Updatable>()
    private val autonomousStartables = ArrayList<AutonomousStartable>()
    fun addUpdatable(updatable:Updatable) = updatables.add(updatable)
    fun addAutonomousStartable(autonomousStartable: AutonomousStartable) = autonomousStartables.add(autonomousStartable)

    fun autoStart(){

    }
    fun autoEnd(){

    }
    fun opModeEnd(){

    }
    fun autoPostInit(){

    }
    fun teleOpPostInit(){

    }
    fun update() = updatables.forEach { it.update() }
}
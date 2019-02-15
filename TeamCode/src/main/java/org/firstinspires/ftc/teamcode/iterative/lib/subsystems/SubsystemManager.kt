package org.firstinspires.ftc.teamcode.iterative.lib.subsystems

class SubsystemManager {
    private val updatables = ArrayList<Updatable>()

    private val autonomousPostInitables = ArrayList<AutonomousPostInitable>()
    private val autonomousStartables = ArrayList<AutonomousStartable>()
    private val autonomousEndables = ArrayList<AutonomousEndable>()

    private val teleOpPostInitables = ArrayList<TeleOpPostInitable>()
    private val teleOpStartables = ArrayList<TeleOpStartable>()
    private val teleOpEndables = ArrayList<TeleOpEndable>()

    fun addUpdatable(updatable: Updatable) = updatables.add(updatable)

    fun addAutonomousPostInitable(autonomousPostInitable: AutonomousPostInitable) = autonomousPostInitables.add(autonomousPostInitable)
    fun addAutonomousStartable(autonomousStartable: AutonomousStartable) = autonomousStartables.add(autonomousStartable)
    fun addAutonomousEndable(autonomousEndable: AutonomousEndable) = autonomousEndables.add(autonomousEndable)

    fun addTeleOpPostInitable(teleOpPostInitable: TeleOpPostInitable) = teleOpPostInitables.add(teleOpPostInitable)
    fun addTeleOpStartable(teleOpStartable: TeleOpStartable) = teleOpStartables.add(teleOpStartable)
    fun addTeleOpEndable(teleOpEndable: TeleOpEndable) = teleOpEndables.add(teleOpEndable)

    fun autoPostInit() = autonomousPostInitables.forEach { it.autoPostInit() }
    fun autoStart() = autonomousStartables.forEach { it.autonomousStart() }
    fun autoEnd() = autonomousEndables.forEach { it.autoEnd() }

    fun teleOpPostInit() = teleOpPostInitables.forEach { it.teleOpPostInit() }
    fun teleOpStart() = teleOpStartables.forEach { it.teleOpStart() }
    fun teleOpEnd() = teleOpEndables.forEach { it.teleOpEnd() }

    fun update() = updatables.forEach { it.update() }
}
package org.firstinspires.ftc.teamcode.iterative.lib.subsystems

class SubsystemManager {
    private val updatables = ArrayList<Updatable>()

    private val autonomousPostInitables = ArrayList<AutonomousPostInitable>()
    private val autonomousStartables = ArrayList<AutonomousStartable>()
    private val autonomousEndables = ArrayList<AutonomousEndable>()

    private val teleOpPostInitables = ArrayList<TeleOpPostInitable>()
    private val teleOpStartables = ArrayList<TeleOpStartable>()
    private val teleOpEndables = ArrayList<TeleOpEndable>()

    @Throws(InterruptedException::class)
    fun addUpdatable(updatable: Updatable) = updatables.add(updatable)
    @Throws(InterruptedException::class)
    fun addAutonomousPostInitable(autonomousPostInitable: AutonomousPostInitable) = autonomousPostInitables.add(autonomousPostInitable)
    @Throws(InterruptedException::class)
    fun addAutonomousStartable(autonomousStartable: AutonomousStartable) = autonomousStartables.add(autonomousStartable)
    @Throws(InterruptedException::class)
    fun addAutonomousEndable(autonomousEndable: AutonomousEndable) = autonomousEndables.add(autonomousEndable)
    @Throws(InterruptedException::class)
    fun addTeleOpPostInitable(teleOpPostInitable: TeleOpPostInitable) = teleOpPostInitables.add(teleOpPostInitable)
    @Throws(InterruptedException::class)
    fun addTeleOpStartable(teleOpStartable: TeleOpStartable) = teleOpStartables.add(teleOpStartable)
    @Throws(InterruptedException::class)
    fun addTeleOpEndable(teleOpEndable: TeleOpEndable) = teleOpEndables.add(teleOpEndable)
    @Throws(InterruptedException::class)
    fun autoPostInit() = autonomousPostInitables.forEach { it.autoPostInit() }
    @Throws(InterruptedException::class)
    fun autoStart() = autonomousStartables.forEach { it.autonomousStart() }
    @Throws(InterruptedException::class)
    fun autoEnd() = autonomousEndables.forEach { it.autoEnd() }
    @Throws(InterruptedException::class)
    fun teleOpPostInit() = teleOpPostInitables.forEach { it.teleOpPostInit() }
    @Throws(InterruptedException::class)
    fun teleOpStart() = teleOpStartables.forEach { it.teleOpStart() }
    @Throws(InterruptedException::class)
    fun teleOpEnd() = teleOpEndables.forEach { it.teleOpEnd() }
    @Throws(InterruptedException::class)
    fun update() = updatables.forEach { it.update() }
}
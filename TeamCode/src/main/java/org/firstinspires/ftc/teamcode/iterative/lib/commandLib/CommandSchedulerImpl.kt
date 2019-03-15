package org.firstinspires.ftc.teamcode.iterative.lib.commandLib

class CommandSchedulerImpl : CommandScheduler {

    private val runningCommands = HashSet<Command>()
    @Throws(InterruptedException::class)
    override fun run(command: Command) {
        command.start()
        runningCommands.add(command)
    }

    @Throws(InterruptedException::class)
    override fun periodic() {
        runningCommands.forEach { command ->
            if (command.isComplete())
                command.end()
            else
                command.periodic()
        }
        runningCommands.removeAll { it.isComplete() }
    }

    @Throws(InterruptedException::class)
    fun isRunningCommands() = !runningCommands.isEmpty()
}
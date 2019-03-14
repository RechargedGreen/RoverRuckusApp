package org.firstinspires.ftc.teamcode.iterative.lib.commandLib

class CommandSchedulerImpl : CommandScheduler {

    private val runningCommands = HashSet<Command>()

    override fun run(command: Command) {
        command.start()
        runningCommands.add(command)
    }

    override fun periodic() {
        runningCommands.forEach { command ->
            if (command.isComplete())
                command.end()
            else
                command.periodic()
        }
        runningCommands.removeAll { it.isComplete() }
    }

    fun isRunningCommands() = !runningCommands.isEmpty()
}
package org.firstinspires.ftc.teamcode.iterative.lib.commandLib

class ParallelCommandGroup(private vararg val commands: Command) : Command {
    private val completedCommands = HashSet<Command>()

    override fun start() = commands.forEach { it.start() }

    override fun periodic() {
        for (command in commands) {
            if (command.isComplete()) {
                if (!completedCommands.contains(command)) {
                    command.end()
                    completedCommands.add(command)
                }
            } else {
                command.periodic()
            }
        }
    }

    override fun isComplete() = commands.all { it.isComplete() }

    override fun end() = commands.forEach {
        if (!completedCommands.contains(it)) it.end()
    }
}
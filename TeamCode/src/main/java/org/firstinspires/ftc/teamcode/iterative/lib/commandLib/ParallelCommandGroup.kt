package org.firstinspires.ftc.teamcode.iterative.lib.commandLib

class ParallelCommandGroup(private vararg val commands: Command) : Command {
    private val completedCommands = HashSet<Command>()
    @Throws(InterruptedException::class)
    override fun start() = commands.forEach { it.start() }
    @Throws(InterruptedException::class)
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
    @Throws(InterruptedException::class)
    override fun isComplete() = commands.all { it.isComplete() }
    @Throws(InterruptedException::class)
    override fun end() = commands.forEach {
        if (!completedCommands.contains(it)) it.end()
    }
}
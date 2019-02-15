package org.firstinspires.ftc.teamcode.iterative.lib.commandLib

class SequentialCommandGroup(private vararg val commands: Command) : Command {
    private var currentIndex = 0

    override fun start() = commands[0].start()

    override fun periodic() {
        if (currentIndex < commands.size) {
            val currentCommand = commands[currentIndex]

            if (currentCommand.isComplete()) {
                currentCommand.end()
                currentIndex++
                if (currentIndex < commands.size)
                    commands[currentIndex].start()
            } else {
                currentCommand.periodic()
            }
        }
    }

    override fun isComplete(): Boolean = currentIndex >= commands.size

    override fun end() = commands[currentIndex].end()
}
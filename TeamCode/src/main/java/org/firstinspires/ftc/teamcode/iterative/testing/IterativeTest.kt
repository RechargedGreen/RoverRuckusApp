package org.firstinspires.ftc.teamcode.iterative.testing

import org.firstinspires.ftc.teamcode.iterative.lib.IterativeOpMode
import org.firstinspires.ftc.teamcode.iterative.lib.commandLib.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.iterative.lib.commandLib.commands.DelayCommand

class IterativeTest : IterativeOpMode(false) {
    override fun onStart() =
            commandScheduler.run(SequentialCommandGroup(
                    DeployCommand(),
                    DriveForwardCommand(0.0, 1000),
                    TurnCommand(90.0),
                    DelayCommand(2.5),
                    DriveForwardCommand(90.0, 1000)
            ))
}
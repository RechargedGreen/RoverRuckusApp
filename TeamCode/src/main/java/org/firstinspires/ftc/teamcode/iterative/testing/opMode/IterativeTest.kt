package org.firstinspires.ftc.teamcode.iterative.testing.opMode

import org.firstinspires.ftc.teamcode.iterative.lib.IterativeOpMode
import org.firstinspires.ftc.teamcode.iterative.lib.commandLib.ParallelCommandGroup
import org.firstinspires.ftc.teamcode.iterative.lib.commandLib.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.iterative.lib.commandLib.commands.DelayCommand
import org.firstinspires.ftc.teamcode.iterative.testing.bot.IterativeBot
import org.firstinspires.ftc.teamcode.iterative.testing.commands.DeployCommand
import org.firstinspires.ftc.teamcode.iterative.testing.commands.DriveForwardCommand
import org.firstinspires.ftc.teamcode.iterative.testing.commands.PIDTurnCommand

class IterativeTest : IterativeOpMode<IterativeBot>(IterativeBot, true) {
    override fun onStart() =
            commandScheduler.run(SequentialCommandGroup(
                    DeployCommand(),
                    ParallelCommandGroup(
                            DriveForwardCommand(0.0, 1000),
                            DelayCommand(4.0)
                    ),
                    PIDTurnCommand(90.0, PIDTurnCommand.TurnType.POINT_TURN),
                    DelayCommand(2.5),
                    DriveForwardCommand(90.0, 1000)
            ))
}
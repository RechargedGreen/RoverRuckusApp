package org.firstinspires.ftc.teamcode.iterative.testing.opMode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.iterative.lib.IterativeOpMode
import org.firstinspires.ftc.teamcode.iterative.lib.commandLib.ParallelCommandGroup
import org.firstinspires.ftc.teamcode.iterative.lib.commandLib.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.iterative.lib.commandLib.commands.DelayCommand
import org.firstinspires.ftc.teamcode.iterative.testing.bot.IterativeBot
import org.firstinspires.ftc.teamcode.iterative.testing.commands.PIDTurnCommand
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(group = OpModeGroups.TELE_DIAGNOSTICS)
class IterativeTest : IterativeOpMode<IterativeBot>(IterativeBot, true) {
    override fun onStart() =
            commandScheduler.run(SequentialCommandGroup(
                    ParallelCommandGroup(
                            PIDTurnCommand(90.0, PIDTurnCommand.TurnType.POINT_TURN),
                            DelayCommand(5.0)
                    ),
                    PIDTurnCommand(0.0, PIDTurnCommand.TurnType.POINT_TURN),
                    DelayCommand(1.0),
                    PIDTurnCommand(90.0, PIDTurnCommand.TurnType.AROUND_LEFT_WHEELS),
                    PIDTurnCommand(0.0, PIDTurnCommand.TurnType.AROUND_LEFT_WHEELS),
                    PIDTurnCommand(-90.0, PIDTurnCommand.TurnType.AROUND_RIGHT_WHEELS),
                    PIDTurnCommand(0.0, PIDTurnCommand.TurnType.AROUND_RIGHT_WHEELS)
            ))
}
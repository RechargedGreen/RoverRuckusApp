package org.firstinspires.ftc.teamcode.iterative.testing.opMode

import com.acmerobotics.roadrunner.Pose2d
import org.firstinspires.ftc.teamcode.iterative.lib.IterativeOpMode
import org.firstinspires.ftc.teamcode.iterative.testing.bot.IterativeBot
import org.firstinspires.ftc.teamcode.iterative.testing.commands.FollowTrajectoryCommand

/**
 * Created by David Lukens on 2/15/2019.
 */
class CommandBasedMotionProfileTest : IterativeOpMode<IterativeBot>(IterativeBot, true) {
    @Throws(InterruptedException::class)
    override fun onStart() = commandScheduler.run(
            FollowTrajectoryCommand(
                    {
                        bot.drive.trajectoryBuilder()
                                .splineTo(Pose2d(24.0, 24.0, Math.toRadians(90.0)))
                                .turn(Math.toRadians(90.0))
                                .turn(Math.toRadians(0.0))
                                .back(24.0)
                                .build()
                    }
            )
    )
}
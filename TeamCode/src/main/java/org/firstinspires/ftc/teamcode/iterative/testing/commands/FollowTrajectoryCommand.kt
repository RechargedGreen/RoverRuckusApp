package org.firstinspires.ftc.teamcode.iterative.testing.commands

import com.acmerobotics.roadrunner.trajectory.Trajectory
import org.firstinspires.ftc.teamcode.iterative.lib.commandLib.Command
import org.firstinspires.ftc.teamcode.iterative.testing.bot.IterativeBot

/**
 * Created by David Lukens on 2/15/2019.
 */
class FollowTrajectoryCommand(private val createTrajectory: () -> Trajectory) : Command {
    init {
        requireNotNull(IterativeBot.drive)
    }

    val drive = IterativeBot.drive
    @Throws(InterruptedException::class)
    override fun start() = drive.trajectoryFollower.followTrajectory(createTrajectory())

    @Throws(InterruptedException::class)
    override fun periodic() = drive.trajectoryFollower.update(drive.poseEstimate)

    @Throws(InterruptedException::class)
    override fun isComplete(): Boolean = !drive.trajectoryFollower.isFollowing()

    @Throws(InterruptedException::class)
    override fun end() = drive.stop()
}
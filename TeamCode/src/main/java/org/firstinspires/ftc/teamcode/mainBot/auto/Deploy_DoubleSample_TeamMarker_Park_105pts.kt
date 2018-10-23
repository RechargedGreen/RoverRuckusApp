package org.firstinspires.ftc.teamcode.mainBot.auto

import com.acmerobotics.dashboard.config.Config
import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.data.FIELD_POSITIONS
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.hardware.SuperSystem
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

/**
 * Created by David Lukens on 10/23/2018.
 */
@Config
@Autonomous(group = OpModeGroups.MAIN_AUTO)
class Deploy_DoubleSample_TeamMarker_Park_105pts : FluidAuto<HardwareClass>({ opMode -> HardwareClass(opMode) }) {
    override fun run() {
        robot.drive.imu.setZBias(FIELD_POSITIONS.SILVER_HANG_ANGLE)
        robot.lift.deploy()

        robot.superSystem.sample(SuperSystem.SampleSituation.LANDER_SILVER)

        robot.drive.waitOnTrajectory(trajectory = robot.drive.trajectoryBuilder()
                .turnTo(FIELD_POSITIONS.ANGLE_BEFORE_WALL_ALIGN_SPLINE_SILVER_SAMPLE_TO_DEPOT)
                .splineTo(FIELD_POSITIONS.ALIGN_WALL_FOLLOW_SILVER_SAMPLE_TO_DEPOT)
                .build())
        robot.drive.followWall(DriveTerrain.WALL_FOLLOWS.OWN_CRATER_TO_DEPOT)
        sleep(1000)

        robot.superSystem.sample(SuperSystem.SampleSituation.DEPOT_GOLD)

        robot.drive.waitOnTrajectory(trajectory = robot.drive.trajectoryBuilder()
                .turnTo(FIELD_POSITIONS.ANGLE_BEFORE_WALL_ALIGN_SPLINE_DEPOT_SAMPLE_TO_CRATER)
                .splineTo(FIELD_POSITIONS.ALIGN_WALL_FOLLOW_DEPOT_SAMPLE_TO_CRATER)
                .build())
        robot.drive.followWall(DriveTerrain.WALL_FOLLOWS.DEPOT_TO_OWN_CRATER)
    }
}
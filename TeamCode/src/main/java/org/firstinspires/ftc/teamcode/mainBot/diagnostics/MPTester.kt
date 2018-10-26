package org.firstinspires.ftc.teamcode.mainBot.diagnostics

import com.acmerobotics.roadrunner.Pose2d
import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(group = OpModeGroups.TELE_DIAGNOSTICS)
class MPTester : FluidAuto<HardwareClass>({ opMode -> HardwareClass(opMode) }) {
    override fun run() {
        robot.drive.waitOnTrajectory(action = Runnable {
            telemetry.addData("PoseEstimate", robot.drive.poseEstimate)
            telemetry.update()
        }, trajectory = robot.drive.trajectoryBuilder()
                .splineTo(Pose2d(24.0, 24.0, 0.0))
                .waitFor(2.0)
                .turn(Math.toRadians(180.0))
                .waitFor(2.0)
                .turn(Math.toRadians(0.0))
                .waitFor(2.0)
                .back(24.0)
                .build()
        )
        loopWhile { opModeIsActive() }
    }
}
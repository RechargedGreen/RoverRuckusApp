package org.firstinspires.ftc.teamcode.prototypes

import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector
import org.firstinspires.ftc.teamcode.data.FIELD_POSITIONS
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass

class SilverSampleTest : FluidAuto<HardwareClass>({ opMode -> HardwareClass(opMode) }) {
    @JvmField var sampleOrder:SamplingOrderDetector.GoldLocation = SamplingOrderDetector.GoldLocation.LEFT
    override fun run() {
        robot.drive.imu.setZBias(FIELD_POSITIONS.SILVER_HANG_ANGLE)
        robot.drive.poseEstimate = FIELD_POSITIONS.SILVER_DEPLOY
        robot.drive.startFollowingTrajectory(robot.drive.trajectoryBuilder().
                splineTo(FIELD_POSITIONS.SILVER_SAMPLE_FROM_LANDER_GENERAL).
                splineTo(when(sampleOrder){
                    SamplingOrderDetector.GoldLocation.UNKNOWN -> FIELD_POSITIONS.SILVER_SAMPLE_FROM_LANDER_CENTER
                    SamplingOrderDetector.GoldLocation.LEFT -> FIELD_POSITIONS.SILVER_SAMPLE_FROM_LANDER_LEFT
                    SamplingOrderDetector.GoldLocation.CENTER -> FIELD_POSITIONS.SILVER_SAMPLE_FROM_LANDER_CENTER
                    SamplingOrderDetector.GoldLocation.RIGHT -> FIELD_POSITIONS.SILVER_SAMPLE_FROM_LANDER_RIGHT
                })
                .build())
        robot.drive.waitOnFollower()

        robot.intake.collectSample()
        sleep(2000)// sleep for testing until sample collection is done

        robot.drive.startFollowingTrajectory(robot.drive.trajectoryBuilder()
                                                     .turnTo(FIELD_POSITIONS.ANGLE_BEFORE_WALL_ALIGN_SPLINE_SILVER_SAMPLE_TO_DEPOT)
                                                     .splineTo(FIELD_POSITIONS.ALIGN_WALL_FOLLOW_SILVER_SAMPLE_TO_DEPOT)
                                                     .build())
        robot.drive.waitOnFollower()
    }
}
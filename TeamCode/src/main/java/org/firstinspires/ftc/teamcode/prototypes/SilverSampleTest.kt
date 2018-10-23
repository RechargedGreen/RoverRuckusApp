package org.firstinspires.ftc.teamcode.prototypes

import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector
import org.firstinspires.ftc.teamcode.data.FIELD_POSITIONS
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass

class SilverSampleTest : FluidAuto<HardwareClass>({ opMode -> HardwareClass(opMode) }) {
    @JvmField var sampleOrder:SamplingOrderDetector.GoldLocation = SamplingOrderDetector.GoldLocation.LEFT
    override fun run() {
        robot.drive.setPos(FIELD_POSITIONS.SILVER_DEPLOY)
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
    }
}
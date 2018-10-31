package org.firstinspires.ftc.teamcode.prototypes

import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector
import org.firstinspires.ftc.teamcode.data.FIELD_POSITIONS
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.hardware.SuperSystem

class SilverSampleTest : FluidAuto<HardwareClass>({ opMode -> HardwareClass(opMode) }) {
    @JvmField var sampleOrder:SamplingOrderDetector.GoldLocation = SamplingOrderDetector.GoldLocation.LEFT
    override fun run() {
        robot.drive.imu.setZBias(FIELD_POSITIONS.SILVER_HANG_ANGLE)
        robot.drive.poseEstimate = FIELD_POSITIONS.SILVER_DEPLOY

        robot.superSystem.sample(SuperSystem.SampleSituation.LANDER_SILVER)
    }
}
package org.firstinspires.ftc.teamcode.mainBot.auto

import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.data.FIELD_POSITIONS
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.hardware.SuperSystem
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

/**
 * Created by David Lukens on 11/2/2018.
 */
@Autonomous(group = OpModeGroups.AUTO)
class Deploy_SingleSample : FluidAuto<HardwareClass>( {opMode -> HardwareClass(opMode) } ){
    override fun run() {
        robot.drive.imu.setZBias(FIELD_POSITIONS.SILVER_HANG_ANGLE, AngleUnit.RADIANS)
        robot.drive.poseEstimate = FIELD_POSITIONS.SILVER_DEPLOY
        robot.lift.deploy()
        robot.superSystem.sample(SuperSystem.SampleSituation.LANDER_SILVER)
    }
}
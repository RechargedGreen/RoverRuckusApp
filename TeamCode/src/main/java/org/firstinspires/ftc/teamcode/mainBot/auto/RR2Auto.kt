package org.firstinspires.ftc.teamcode.mainBot.auto

import com.acmerobotics.dashboard.config.Config
import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions

/**
 * Created by David Lukens on 11/18/2018.
 */
@Config
abstract class RR2Auto(val angleAtStart:Double) : FluidAuto<HardwareClass>({ opMode -> HardwareClass(opMode) }){
    companion object {
        @JvmField var GOLD_HANG_ANGLE = -45.0
        @JvmField var SILVER_HANG_ANGLE = -135.0
    }
    var ORDER = SampleRandomizedPositions.UNKNOWN
    override fun tillStart() {
        ORDER = robot.vision.tfLite.lastKnownSampleOrder
        telemetry.addData("Order", ORDER)
        telemetry.update()
    }

    override fun run() {
        robot.drive.imu.setZ(angleAtStart, AngleUnit.DEGREES)
        robot.lift.deploy()
    }

    abstract fun postDeploy()
}
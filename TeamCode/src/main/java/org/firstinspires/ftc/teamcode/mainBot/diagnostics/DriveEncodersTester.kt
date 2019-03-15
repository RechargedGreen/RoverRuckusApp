package org.firstinspires.ftc.teamcode.mainBot.diagnostics

import com.david.rechargedkotlinlibrary.internal.opMode.PracticeTeleOp
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(group = OpModeGroups.TELE_DIAGNOSTICS)
class DriveEncodersTester : PracticeTeleOp<HardwareClass>({ opMode -> HardwareClass(opMode) }) {
    @Throws(InterruptedException::class)
    override fun onStart() {
        super.onStart()
        robot.drive.imu.setZBias(45.0, AngleUnit.DEGREES)
    }

    @Throws(InterruptedException::class)
    override fun onLoop() {
        telemetry.addData("lf", robot.getHub(0).getEncoder(0))
        telemetry.addData("lb", robot.getHub(0).getEncoder(1))
        telemetry.addData("rf", robot.getHub(0).getEncoder(2))
        telemetry.addData("rb", robot.getHub(0).getEncoder(3))
        telemetry.addData("left", robot.drive.leftRawTicks())
        telemetry.addData("right", robot.drive.rightRawTicks())
        robot.drive.openLoopPowerWheels(c1.ly, c1.ry)
        telemetry.addData("lp", c1.ly)
        telemetry.addData("rp", c1.ry)
        telemetry.addData("raw z", robot.drive.imu.getRawZ(AngleUnit.DEGREES))
        telemetry.addData("z", robot.drive.imu.getZ(AngleUnit.DEGREES))
    }
}
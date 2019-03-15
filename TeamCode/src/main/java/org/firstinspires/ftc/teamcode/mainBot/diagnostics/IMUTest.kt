package org.firstinspires.ftc.teamcode.mainBot.diagnostics

import com.david.rechargedkotlinlibrary.internal.opMode.PracticeTeleOp
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(group = OpModeGroups.TELE_DIAGNOSTICS)
class IMUTest : PracticeTeleOp<HardwareClass>({ opMode -> HardwareClass(opMode) }) {
    @Throws(InterruptedException::class)
    override fun onLoop() {
        telemetry.addData("raw z", robot.drive.imu.getRawZ())
        telemetry.addData("raw x", robot.drive.imu.getRawX())
        telemetry.addData("raw y", robot.drive.imu.getRawY())
        telemetry.addData("z", robot.drive.imu.getZ())
        telemetry.addData("x", robot.drive.imu.getX())
        telemetry.addData("y", robot.drive.imu.getY())
        if (c1.x) robot.drive.imu.resetX()
        telemetry.addLine("x to reset x")
        if (c1.y) robot.drive.imu.resetY()
        telemetry.addLine("y to reset y")
        if (c1.b) robot.drive.imu.resetZ()
        telemetry.addLine("b to reset z")
    }
}
package org.firstinspires.ftc.teamcode.mainBot.tuner

import com.david.rechargedkotlinlibrary.internal.hardware.PIDController
import com.david.rechargedkotlinlibrary.internal.opMode.PracticeTeleOp
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.PIDCoefficients
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(group = OpModeGroups.TELE_MISC)
class PIDCorrectionTest : PracticeTeleOp<HardwareClass>( {opMode -> HardwareClass(opMode) } ){
    override fun onLoop() {
        if(c1.y)
            robot.drive.startFollowingAngle(angle = 0.0, controller = PIDController(coeffs = PIDCoefficients(0.1, 0.0, 0.0)), power = 0.0)
        if(c1.x)
            robot.drive.startFollowingAngle(angle = 0.0, controller = PIDController(coeffs = PIDCoefficients(0.0, 0.001, 0.0)), power = 0.0)
        if(c1.a)
            robot.drive.startFollowingAngle(angle = 0.0, controller = PIDController(coeffs = PIDCoefficients(0.0, 0.0, 0.5)), power = 0.0)
        if(c1.b)
            robot.drive.stop()
        telemetry.addLine("y for p")
        telemetry.addLine("x for i")
        telemetry.addLine("a for d")
        telemetry.addLine("b to stop")
        packet.put("error", robot.drive.lastAngleFollowerError)
    }
}
package org.firstinspires.ftc.teamcode.prototypes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups
import kotlin.math.absoluteValue

@TeleOp(group = OpModeGroups.TELE_MISC)
class LiftTest : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val l = hardwareMap.dcMotor.get("liftL")
        val r = hardwareMap.dcMotor.get("liftR")
        l.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        r.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        r.direction = DcMotorSimple.Direction.REVERSE
        waitForStart()
        while (opModeIsActive()) {
            var p = -gamepad1.left_stick_y.toDouble()
            if (p.absoluteValue < 0.1) p = 0.0
            l.power = p
            r.power = p
        }
    }
}
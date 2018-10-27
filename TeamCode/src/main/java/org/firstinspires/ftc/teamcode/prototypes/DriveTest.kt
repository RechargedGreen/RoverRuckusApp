package org.firstinspires.ftc.teamcode.prototypes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups
import kotlin.math.absoluteValue

@TeleOp(group = OpModeGroups.TELE_DIAGNOSTICS)
class DriveTest : LinearOpMode() {
    val deadBand = 0.05
    val turnSensitivity = 1.0
    override fun runOpMode() {
        val lf = hardwareMap.dcMotor.get("lf")
        val lb = hardwareMap.dcMotor.get("lb")
        val rf = hardwareMap.dcMotor.get("rf")
        val rb = hardwareMap.dcMotor.get("rb")
        lf.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        lb.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rf.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rb.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rf.direction = DcMotorSimple.Direction.REVERSE
        rb.direction = DcMotorSimple.Direction.REVERSE
        waitForStart()
        while (opModeIsActive()) {
            var left = -gamepad1.left_stick_y.toDouble()
            var right = -gamepad1.right_stick_y.toDouble()
            left *= (1.0 - gamepad1.left_trigger * turnSensitivity)
            right *= (1.0 - gamepad1.right_trigger * turnSensitivity)
            if (left.absoluteValue <= deadBand) left = 0.0
            if (right.absoluteValue <= deadBand) right = 0.0
            lf.power = left
            lb.power = left
            rf.power = right
            rb.power = right
            telemetry.addData("left power", left)
            telemetry.addData("right power", right)
            telemetry.addData("lf", lf.currentPosition)
            telemetry.addData("lb", lb.currentPosition)
            telemetry.addData("rf", rf.currentPosition)
            telemetry.addData("rb", rb.currentPosition)
            telemetry.update()
        }
    }
}
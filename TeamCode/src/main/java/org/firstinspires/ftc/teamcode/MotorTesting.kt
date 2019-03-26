package org.firstinspires.ftc.teamcode

import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple

@TeleOp
class MotorTesting : LinearOpMode() {
    override fun runOpMode() {
        val lf = HardwareMaker.DcMotorEx.make(hardwareMap, "lf")
        val lb = HardwareMaker.DcMotorEx.make(hardwareMap, "lb")
        val rf = HardwareMaker.DcMotorEx.make(hardwareMap, "rf", DcMotorSimple.Direction.REVERSE)
        val rb = HardwareMaker.DcMotorEx.make(hardwareMap, "rb", DcMotorSimple.Direction.REVERSE)

        val liftL = HardwareMaker.DcMotorEx.make(hardwareMap, "liftL", DcMotorSimple.Direction.REVERSE)
        val liftR = HardwareMaker.DcMotorEx.make(hardwareMap, "liftR")

        val extension = HardwareMaker.DcMotorEx.make(hardwareMap, "extension", DcMotorSimple.Direction.REVERSE)
        val intake = HardwareMaker.DcMotorEx.make(hardwareMap, "intake", DcMotorSimple.Direction.REVERSE)

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addLine("gamepad1 leftY for leftSide")
            telemetry.addLine("gamepad1 rightY for rightSide")
            telemetry.addLine("gamepad1 triggers for lift")
            telemetry.addLine("gamepad2 leftY for intake")
            telemetry.addLine("gamepad2 rightY for extension")

            val leftSignal = -gamepad1.left_stick_y.toDouble()
            val rightSignal = -gamepad1.right_stick_y.toDouble()

            val liftSignal = (gamepad1.right_trigger - gamepad1.left_trigger).toDouble()

            val intakeSignal = -gamepad2.left_stick_y.toDouble()
            val extensionSignal = -gamepad2.right_stick_y.toDouble()

            lf.power = leftSignal
            lb.power = leftSignal
            rf.power = rightSignal
            rb.power = rightSignal

            liftL.power = liftSignal
            liftR.power = liftSignal

            intake.power = intakeSignal
            extension.power = extensionSignal

            telemetry.update()
        }
    }
}
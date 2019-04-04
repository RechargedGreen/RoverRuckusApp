package org.firstinspires.ftc.teamcode.mainBot.tuner

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@Config
@TeleOp(group = OpModeGroups.TELE_TUNERS)
class BucketTuner : LinearOpMode() {
    lateinit var servo:Servo
    override fun runOpMode() {
        servo = hardwareMap.servo.get("flip")
        waitForStart()
        while (opModeIsActive()) setPosition(target)
    }

    fun setPosition(position: Double) {
        servo.position = position
    }

    companion object {
        @JvmField
        var target: Double = 0.0
    }
}

@Config
@TeleOp(group = OpModeGroups.TELE_TUNERS)
class LatchTuner : LinearOpMode() {
    lateinit var servo: Servo
    override fun runOpMode() {
        servo = hardwareMap.servo.get("latch")
        waitForStart()
        while (opModeIsActive()) setPosition(target)
    }

    fun setPosition(position: Double) {
        servo.position = position
    }

    companion object {
        @JvmField
        var target: Double = 0.0
    }
}

@Config
@TeleOp(group = OpModeGroups.TELE_TUNERS)
class FlipTuner : LinearOpMode() {
    lateinit var left:Servo
    lateinit var right:Servo
    override fun runOpMode() {
        left = hardwareMap.servo.get("intakeFlipL")
        right = hardwareMap.servo.get("intakeFlipR")
        waitForStart()
        while (opModeIsActive()) setPosition(target)
    }

    fun setPosition(position: Double) {
        left.position = 1.0 - position
        right.position = position
    }

    companion object {
        @JvmField
        var target: Double = 0.0
    }
}
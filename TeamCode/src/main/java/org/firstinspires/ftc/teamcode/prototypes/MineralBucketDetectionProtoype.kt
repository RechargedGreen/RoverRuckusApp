package org.firstinspires.ftc.teamcode.prototypes

import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@TeleOp
class MineralBucketDetectionProtoype : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val color = hardwareMap.colorSensor.get("color")
        val tof = hardwareMap.get(DistanceSensor::class.java, "tof") as Rev2mDistanceSensor
        waitForStart()
        while (opModeIsActive()) {
            telemetry.addData("alpha", color.alpha())
            telemetry.addData("red", color.red())
            telemetry.addData("green", color.green())
            telemetry.addData("blue", color.blue())
            telemetry.addLine()
            telemetry.addData("INCH", tof.getDistance(DistanceUnit.INCH))
            telemetry.addData("CM", tof.getDistance(DistanceUnit.CM))
            telemetry.addData("METER", tof.getDistance(DistanceUnit.METER))
            telemetry.addData("MM", tof.getDistance(DistanceUnit.MM))

            telemetry.update()
        }
    }
}
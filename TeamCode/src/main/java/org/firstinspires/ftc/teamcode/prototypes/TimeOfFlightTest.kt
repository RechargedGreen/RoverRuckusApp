package org.firstinspires.ftc.teamcode.prototypes

import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(group = OpModeGroups.TELE_DIAGNOSTICS)
class TimeOfFlightTest : LinearOpMode() {
    override fun runOpMode() {
        val sensor = hardwareMap.get(DistanceSensor::class.java, "tof") as Rev2mDistanceSensor
        waitForStart()
        while (opModeIsActive()) {
            telemetry.addData("distance in INCH", sensor.getDistance(DistanceUnit.INCH))
            telemetry.update()
        }
    }
}
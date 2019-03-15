package org.firstinspires.ftc.teamcode.prototypes

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups
import java.util.*

@TeleOp(name = "ServoTester", group = OpModeGroups.TELE_DIAGNOSTICS)
@Config
class ServosTesting : LinearOpMode() {
    companion object {
        @JvmField
        var position: Double = 0.0
    }

    private val servos = LinkedList<Servo>()
    private var lastBState = false
    private var lastAState = false
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        while (!(isStarted || isStopRequested)) {
            telemetry.addLine("a adds servo")
            telemetry.addLine("b removes servo")
            telemetry.addLine("x reverses servo")
            telemetry.addLine("y sets servo forward")
            telemetry.addData("servo count", servos.size)
            telemetry.update()
            if (gamepad1.a) {
                if (!lastAState) try {
                    servos.add(hardwareMap.servo.get("s${servos.size}"))
                } catch (ex: IllegalArgumentException) {
                }
                lastAState = true
            } else lastAState = false
            if (gamepad1.b) {
                if (!lastBState && !servos.isEmpty())
                    servos.removeLast()
                lastBState = true
            } else lastBState = false
            if (gamepad1.x && !servos.isEmpty()) servos.get(servos.size - 1).direction = Servo.Direction.REVERSE
            if (gamepad1.y && !servos.isEmpty()) servos.get(servos.size - 1).direction = Servo.Direction.FORWARD
        }
        waitForStart()
        while (opModeIsActive()) {
            val pos = position
            if (pos != null)
                servos.forEach { it.position = pos }
            telemetry.addData("position", position)
            telemetry.update()
        }
    }
}
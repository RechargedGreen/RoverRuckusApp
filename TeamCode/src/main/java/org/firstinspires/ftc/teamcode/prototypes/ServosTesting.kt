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
    @JvmField
    var position: Double? = null
    private val servos = LinkedList<Servo>()
    private var lastBState = false
    private var lastAState = false
    override fun runOpMode() {
        while (!(isStarted || isStopRequested)) {
            if (gamepad1.a) {
                if (!lastAState)
                    servos.add(hardwareMap.servo.get("s${servos.size}"))
                lastAState = true
            } else lastAState = false
            if (gamepad1.b) {
                if (!lastBState)
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
        }
    }
}
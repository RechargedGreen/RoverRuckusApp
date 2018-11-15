package org.firstinspires.ftc.teamcode.mainBot.tuner

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.david.rechargedkotlinlibrary.internal.hardware.PIDController
import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import com.david.rechargedkotlinlibrary.internal.opMode.PracticeTeleOp
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.PIDCoefficients
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(group = OpModeGroups.TELE_TUNERS)
@Config
class AngleFollowTuner : FluidAuto<HardwareClass>({ opMode -> HardwareClass(opMode) }) {

    companion object {
        @JvmField
        var currKP = 0.0
        @JvmField
        var currKI = 0.0
        @JvmField
        var currKD = 0.0
        @JvmField
        var currVel = 0.0
        @JvmField
        var currTarget = 0.0
    }

    var following = false

    var lastKP = 0.0
    var lastKI = 0.0
    var lastKD = 0.0
    var lastVel = 0.0
    var lastTarget = 0.0
    override fun run() {
        val dash = FtcDashboard.getInstance()
        val packet = TelemetryPacket()

        loop {
            if (gamepad1.b) {
                robot.drive.stop()
                following = false
            }
            val kP = currKP
            val kI = currKI
            val kD = currKD
            val vel = currVel
            val target = currTarget
            if (lastKP != kP || lastKI != kI || lastKD != kD || vel != currVel || lastTarget != target) {
                lastKP = kP
                lastKI = kI
                lastKD = kD
                lastVel = vel
                lastTarget = target
                robot.drive.startFollowingAngle(PIDController(PIDCoefficients(kP, kI, kD)), power = vel, angle = target)
                following = true
            }

            packet.put("error", robot.drive.lastAngleFollowerError)

            dash.sendTelemetryPacket(packet)
        }
    }
}
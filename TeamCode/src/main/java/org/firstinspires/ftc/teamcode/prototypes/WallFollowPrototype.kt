package org.firstinspires.ftc.teamcode.prototypes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.david.rechargedkotlinlibrary.internal.hardware.PIDController
import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import com.david.rechargedkotlinlibrary.internal.util.BooleanToggle
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.PIDCoefficients
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(group = OpModeGroups.TELE_MISC)
@Config
class WallFollowPrototype : FluidAuto<HardwareClass>({ opMode -> HardwareClass(opMode) }){
    companion object {
        @JvmField
        var kPA = 0.0
        @JvmField
        var kIA = 0.0
        @JvmField
        var kDA = 0.0
        @JvmField
        var kPD = 0.0
        @JvmField
        var kID = 0.0
        @JvmField
        var kDD = 0.0
        @JvmField
        var speed = 0.3
        @JvmField
        var target = 0.0
    }

    var lKPA = 0.0
    var lKIA = 0.0
    var lKDA = 0.0
    var lKPD = 0.0
    var lKID = 0.0
    var lKDD = 0.0

    override fun run() {
        val packet = TelemetryPacket()
        val dash = FtcDashboard.getInstance()
        val followingToggle = BooleanToggle(false)

        var cA = PIDController(PIDCoefficients(kPA, kIA, kDA))
        var cD = PIDController(PIDCoefficients(kPD, kID, kDD))

        while (opModeIsActive()){
            var cKPA = kPA
            var cKIA = kIA
            var cKDA = kDA
            var cKPD = kPD
            var cKID = kID
            var cKDD = kDD

            var a = robot.drive.imu.getZ(AngleUnit.DEGREES)
            var d = robot.sensors.getRightDistanceFromWall(0.0)
            var dError = target - d
            var aError = 0.0
            var aTarget = 0.0

            aTarget = cD.update(dError)
            aError = aTarget - a

            if (cKPA != lKPA || cKIA != lKIA || cKDA != lKDA || cKPD != lKPD || cKID != lKID || cKDD != lKDD){
                cA = PIDController(PIDCoefficients(cKPA, cKIA, cKDA))
                cD = PIDController(PIDCoefficients(cKPD, cKID, cKDD))
            }

            followingToggle.update(gamepad1.a)
            if(followingToggle.toggled()){
                val turn = -cA.update(aError)
                robot.drive.openLoopArcade(speed, turn)
            }else{
                robot.drive.stop()
            }

            packet.put("dError", dError)
            packet.put("aError", aError)
            packet.put("target", target)
            packet.put("a", a)
            packet.put("d", d)
            packet.put("aTarget", aTarget)

            dash.sendTelemetryPacket(packet)
        }
    }
}
package org.firstinspires.ftc.teamcode.mainBot.tuner

import com.david.rechargedkotlinlibrary.internal.opMode.PracticeTeleOp
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass

class AngleFollowTuner:PracticeTeleOp<HardwareClass>({ opMode -> HardwareClass(opMode) }){
    override fun onLoop() {
        telemetry.addLine("press a to start following angle")
        if(gamepad1.a)
            robot.drive.startFollowingAngle_setConstants(DriveTerrain.AngleFollowSpeeds.FAST, 0.0)
        if (gamepad1.b)
            robot.drive.stop()
        telemetry.addLine("press b to stop")
        packet.put("error", robot.drive.lastAngleFollowerError)
    }
}
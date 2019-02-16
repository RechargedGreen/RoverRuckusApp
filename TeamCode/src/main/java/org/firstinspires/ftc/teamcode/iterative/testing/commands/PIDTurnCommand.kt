package org.firstinspires.ftc.teamcode.iterative.testing.commands

import com.david.rechargedkotlinlibrary.internal.hardware.PIDController
import com.david.rechargedkotlinlibrary.internal.util.MathUtil
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.iterative.lib.commandLib.Command
import org.firstinspires.ftc.teamcode.iterative.testing.bot.DriveConstants
import org.firstinspires.ftc.teamcode.iterative.testing.bot.IterativeBot
import kotlin.math.absoluteValue

class PIDTurnCommand(private val targetHeading: Double, private val turnType: TurnType, private val threshold: Double = 2.0) : Command {

    enum class TurnType {
        POINT_TURN,
        AROUND_LEFT_WHEELS,
        AROUND_RIGHT_WHEELS,
        AROUND_FRONT_WHEELS,
        AROUND_BACK_WHEELS,
    }

    init {
        requireNotNull(IterativeBot.drive)
        requireNotNull(IterativeBot.imu)
    }

    private val drive = IterativeBot.drive
    private val imu = IterativeBot.imu
    private val controller = PIDController(DriveConstants.pidConstants)

    override fun start() {}
    override fun periodic() {
        val turn = controller.update(error())
        when (turnType) {
            TurnType.POINT_TURN          -> drive.robotCentric(0.0, 0.0, turn)
            TurnType.AROUND_LEFT_WHEELS  -> drive.robotCentric(-turn, 0.0, turn)
            TurnType.AROUND_RIGHT_WHEELS -> drive.robotCentric(turn, 0.0, turn)
            TurnType.AROUND_FRONT_WHEELS -> drive.robotCentric(0.0, -turn, turn)
            TurnType.AROUND_BACK_WHEELS  -> drive.robotCentric(0.0, turn, turn)
        }
    }

    override fun isComplete() = error().absoluteValue < threshold
    override fun end() = drive.stop()
    private fun error() = MathUtil.norm(imu.getZ(AngleUnit.DEGREES) - targetHeading)
}
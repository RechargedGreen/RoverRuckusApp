package org.firstinspires.ftc.teamcode.mainBot.teleOp

import com.david.rechargedkotlinlibrary.internal.opMode.PracticeTeleOp
import com.david.rechargedkotlinlibrary.internal.util.BooleanToggle
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.hardware.Dumper
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.hardware.Intake
import org.firstinspires.ftc.teamcode.mainBot.hardware.Lift
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups
import kotlin.math.absoluteValue

@TeleOp(name = Practice.NAME, group = OpModeGroups.TELEOP)
open class Practice : PracticeTeleOp<HardwareClass>({ opMode -> HardwareClass(opMode) }) {
    val deadBand = 0.05
    val liftFailSafesToggle = BooleanToggle(true)

    private enum class Mode {
        MANUAL,
        AUTO
    }

    private var liftMode = Mode.AUTO

    override fun onLoop() {
        val l = c1.ly * (1.0 - c1.lt)
        val r = c1.ry * (1.0 - c1.rt)
        robot.drive.openLoopPowerWheels(if (l.absoluteValue > deadBand) l else 0.0, if (r.absoluteValue > deadBand) r else 0.0)

        robot.dumper.state = if (c1.lb && !(c2.dpr && !robot.intake.extensionOut())) Dumper.DumpState.DUMP else Dumper.DumpState.LOAD

        liftFailSafesToggle.update(c2.x)
        val lift = c2.ly
        if (lift.absoluteValue > deadBand || !liftFailSafesToggle.toggled())
            liftMode = Mode.MANUAL
        if (c2.y && liftFailSafesToggle.toggled()) {
            liftMode = Mode.AUTO
        }
        if (liftMode == Mode.MANUAL)
            robot.lift.setOpenLoopPower(if (lift.absoluteValue > deadBand) lift else 0.0, useFailSafes = liftFailSafesToggle.toggled())
        else
            robot.lift.state = if (c2.dpr && !robot.intake.extensionIn()) Lift.State.DOWN else if (c1.rb) Lift.State.UP else Lift.State.DOWN


        if (c2.dp) {
            robot.intake.brakingExtension = true
            robot.intake.flipState = Intake.FlipState.LOAD
            robot.intake.extensionState = Intake.IntakeExtensionState.IN
            robot.intake.intakeState = if (robot.intake.extensionIn()) Intake.IntakeState.IN else Intake.IntakeState.STOP
        } else {
            robot.intake.brakingExtension = false
            robot.intake.intakePower = c2.rt - c2.lt
            robot.intake.manualPowerExtension(c2.ry, true)
            if (c2.rb)
                robot.intake.flipState = Intake.FlipState.LOAD
            if (c2.lb)
                robot.intake.flipState = Intake.FlipState.INTAKE
        }

        telemetry.addData("lift power", lift)
        telemetry.addData("using liftFailSafes", liftFailSafesToggle.toggled())
        telemetry.addData("lift mode", liftMode)
        telemetry.addData("extensionIn", robot.intake.extensionIn())
    }

    companion object {
        const val NAME = "Practice"
    }
}
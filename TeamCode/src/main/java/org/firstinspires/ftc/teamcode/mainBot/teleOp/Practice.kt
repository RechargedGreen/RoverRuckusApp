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
    private enum class Mode{
        MANUAL,
        AUTO
    }

    private var liftMode = Mode.AUTO

    override fun onLoop() {
        val l = c1.ly * (1.0 - c1.lt)
        val r = c1.ry * (1.0 - c1.rt)
        robot.drive.openLoopPowerWheels(if(l.absoluteValue > deadBand) l else 0.0, if(r.absoluteValue > deadBand) r else 0.0)

        robot.dumper.state = if(c1.lb) Dumper.DumpState.DUMP else Dumper.DumpState.LOAD

        liftFailSafesToggle.update(c2.x)
        val lift = c2.ly
        if(lift.absoluteValue > deadBand || !liftFailSafesToggle.toggled())
            liftMode = Mode.MANUAL
        if(c2.y && liftFailSafesToggle.toggled()) {
            liftMode = Mode.AUTO
        }
        if(liftMode == Mode.MANUAL)
            robot.lift.setOpenLoopPower(if(lift.absoluteValue > deadBand) lift else 0.0, useFailSafes = liftFailSafesToggle.toggled())
        else
            robot.lift.state = if(c1.rb) Lift.State.UP else Lift.State.DOWN

        robot.intake.intakeState = if(c2.rb || c2.rt > 0.5) Intake.IntakeState.IN else if (c2.lb || c2.lt > 0.5) Intake.IntakeState.OUT else Intake.IntakeState.STOP
        robot.intake.manualPowerExtension(c2.ry, true)

        if(c2.dpr)
            robot.intake.intakeBucketState = Intake.IntakeBucketState.LOAD_BUCKET
        if(c2.dpu)
            robot.intake.intakeBucketState = Intake.IntakeBucketState.UP
        if(c2.dpd)
            robot.intake.intakeBucketState = Intake.IntakeBucketState.INTAKE

        telemetry.addData("lift power", lift)
        telemetry.addData("using liftFailSafes", liftFailSafesToggle.toggled())
        telemetry.addData("lift mode", liftMode)
    }

    companion object {
        const val NAME = "Practice"
    }
}
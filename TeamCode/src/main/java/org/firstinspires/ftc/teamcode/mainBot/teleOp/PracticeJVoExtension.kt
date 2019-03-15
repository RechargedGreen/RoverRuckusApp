package org.firstinspires.ftc.teamcode.mainBot.teleOp

import com.acmerobotics.dashboard.config.Config
import com.david.rechargedkotlinlibrary.internal.opMode.PracticeTeleOp
import com.david.rechargedkotlinlibrary.internal.util.BooleanToggle
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.mainBot.hardware.Dumper
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.hardware.Intake
import org.firstinspires.ftc.teamcode.mainBot.hardware.Lift
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups
import kotlin.math.absoluteValue

@Config
@TeleOp(name = PracticeJVoExtension.NAME, group = OpModeGroups.TELEOP)
open class PracticeJVoExtension : PracticeTeleOp<HardwareClass>({ opMode -> HardwareClass(opMode) }) {
    val deadBand = 0.05
    val liftFailSafesToggle = BooleanToggle(true)

    private enum class Mode {
        MANUAL,
        AUTO
    }

    private var liftMode = Mode.AUTO

    private var flipState = Intake.FlipState.LOAD

    private var hasBeenLoaded = false

    private var autoRaise = false
    private val hasBeenLoadedTime = ElapsedTime()
    private val hasBeenEmptyTime = ElapsedTime()

    @Throws(InterruptedException::class)
    override fun onLoop() {
        val l = c1.ly
        val r = c1.ry
        robot.drive.openLoopPowerWheels(if (l.absoluteValue > deadBand) l else 0.0, if (r.absoluteValue > deadBand) r else 0.0)

        robot.dumper.state = if (c1.lb && !(c2.dpr && !robot.intake.extensionOut())) (if (c2.a) Dumper.DumpState.SLIGHT_DUMP else Dumper.DumpState.DUMP) else Dumper.DumpState.LOAD

        if(!(robot.superSystem.bucketSense.left() && robot.superSystem.bucketSense.right()))
            hasBeenLoadedTime.reset()
        else
            hasBeenEmptyTime.reset()
        if(hasBeenLoadedTime.seconds() > 0.01 || (c1.lb && robot.lift.getControlState() == Lift.ControlState.AUTO))
            autoRaise = true
        else if(hasBeenEmptyTime.seconds() > 0.25)
            autoRaise = false

        liftFailSafesToggle.update(c2.x)
        val lift = c2.ly
        if (lift.absoluteValue > deadBand || !liftFailSafesToggle.toggled())
            liftMode = Mode.MANUAL
        if ((c1.y || c2.y) && liftFailSafesToggle.toggled()) {
            liftMode = Mode.AUTO
        }

        if (liftMode == Mode.MANUAL)
            robot.lift.setOpenLoopPower(if (lift.absoluteValue > deadBand) lift else 0.0, useFailSafes = liftFailSafesToggle.toggled())
        else
            robot.lift.state = if (c2.dpr && !robot.intake.extensionIn()) Lift.State.DOWN else if (c1.rb || autoRaise) Lift.State.UP else Lift.State.DOWN


        val firstTakesBackExtend = (c1.lt - c1.rt).absoluteValue > 0.2
        if (c2.b && !firstTakesBackExtend) {
            robot.intake.brakingExtension = true
            robot.intake.manualPowerExtension(-holdInPower, false)
            controlA()
        } else if (c2.dpr && !firstTakesBackExtend) {
            robot.intake.brakingExtension = true
            robot.intake.extensionState = Intake.IntakeExtensionState.IN
            controlB()
        } else {
            robot.intake.brakingExtension = false
            robot.intake.manualPowerExtension(c1.rt - c1.lt, true)
            controlA()
        }

        robot.intake.flipState = flipState

        telemetry.addData("lift power", lift)
        telemetry.addData("using liftFailSafes", liftFailSafesToggle.toggled())
        telemetry.addData("lift mode", liftMode)
        telemetry.addData("extensionIn", robot.intake.extensionIn())
    }

    @Throws(InterruptedException::class)
    private fun controlA() {
        robot.intake.intakePower = c2.rt - c2.lt
        if (c2.rb)
            flipState = Intake.FlipState.LOAD
        if (c2.lb)
            flipState = Intake.FlipState.INTAKE
    }

    @Throws(InterruptedException::class)
    private fun controlB() {
        flipState = Intake.FlipState.LOAD
        robot.intake.intakeState = if (robot.intake.extensionIn()) Intake.IntakeState.IN else Intake.IntakeState.STOP
    }

    companion object {
        const val NAME = "PracticeJVoExtension"
        var holdInPower = 0.1
    }
}
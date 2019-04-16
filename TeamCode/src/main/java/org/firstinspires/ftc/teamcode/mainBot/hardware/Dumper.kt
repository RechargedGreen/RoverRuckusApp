package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.acmerobotics.dashboard.config.Config
import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.robotcore.util.ElapsedTime

@Config
class Dumper(val robot: HardwareClass) : MTSubsystem {

    enum class DumpState {
        LOAD,
        DUMP,
        HOLD,
        SLIGHT_DUMP
    }

    companion object {
        // full pwm signal messes with savox servos
        @JvmField
        var loadPos = 0.7
        @JvmField
        var dumpPos = 0.1
        @JvmField
        var holdPos = 0.65
        @JvmField
        var slightDumpPos = 0.3
    }

    private val flip = HardwareMaker.Servo.make(robot.hMap, "flip")

    private var lastState: DumpState? = null

    var state = DumpState.HOLD
        set(value) {
            if (value == DumpState.DUMP && value != field)
                Static.textToSpeech.speak("Epic gamer moment rmao xd")
            else if (value == DumpState.SLIGHT_DUMP || value == DumpState.DUMP)
                dumpTimer.reset()
            lastState = field
            field = value
        }

    val dumpTimer = ElapsedTime()

    @Throws(InterruptedException::class)
    override fun update() {
        when (state) {
            DumpState.LOAD -> internalSetFlipPosition(if (robot.lift.getControlState() != Lift.ControlState.MANUAL_DANGER && (robot.lift.state == Lift.State.UP || !robot.lift.isFullyDown())) holdPos else loadPos)
            DumpState.DUMP, DumpState.SLIGHT_DUMP -> internalSetFlipPosition(if (robot.lift.isFullyUp() || robot.lift.getControlState() == Lift.ControlState.MANUAL_DANGER) (if (state == DumpState.SLIGHT_DUMP) slightDumpPos else dumpPos) else holdPos)
            DumpState.HOLD -> internalSetFlipPosition(holdPos)
        }
    }

    @Throws(InterruptedException::class)
    private fun internalSetFlipPosition(pos: Double) {
        flip.position = pos
    }

    @Throws(InterruptedException::class)
    fun clearingLift() = clearingDown() && clearingUp()

    @Throws(InterruptedException::class)
    fun clearingUp() = true

    @Throws(InterruptedException::class)
    fun clearingDown() = state != DumpState.DUMP && state != DumpState.SLIGHT_DUMP && dumpTimer.seconds() > 0.5

    @Throws(InterruptedException::class)
    fun readyToLoad() = true

    @Throws(InterruptedException::class)
    override fun start() {
    }

    init {
        robot.thread.addSubsystem(this)
    }
}
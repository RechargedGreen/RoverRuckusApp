package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.acmerobotics.dashboard.config.Config
import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedServo
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
        var loadPos = 0.15
        var dumpPos = 1.0
        var holdPos = 0.4
        var slightDumpPos = 0.5
    }

    private val flipL = HardwareMaker.Servo.make(robot.hMap, "flipL")
    private val flipR = HardwareMaker.Servo.make(robot.hMap, "flipR")

    private var lastState:DumpState? = null

    var state = DumpState.LOAD
        set(value){
            if(value == DumpState.DUMP && value != field)
                Static.textToSpeech.speak("Epic gamer moment rmao xd")
            else if(value == DumpState.SLIGHT_DUMP || value == DumpState.DUMP)
                dumpTimer.reset()
            lastState = field
            field = value
        }

    val dumpTimer = ElapsedTime()

    override fun update() {
        when(state) {
            DumpState.LOAD -> internalSetFlipPosition(if(robot.lift.getControlState() != Lift.ControlState.MANUAL_DANGER && (robot.lift.state == Lift.State.UP || !robot.lift.isFullyDown())) holdPos else loadPos)
            DumpState.DUMP, DumpState.SLIGHT_DUMP -> internalSetFlipPosition(if(robot.lift.isFullyUp() || robot.lift.getControlState() == Lift.ControlState.MANUAL_DANGER)(if(state == DumpState.SLIGHT_DUMP) slightDumpPos else dumpPos) else holdPos)
            DumpState.HOLD -> internalSetFlipPosition(holdPos)
        }
    }

    private fun internalSetFlipPosition(pos:Double) {
        flipL.position = pos
        flipR.position = 1.0 - pos
    }

    fun clearingLift() = clearingDown() && clearingUp()
    fun clearingUp() = true
    fun clearingDown() = state != DumpState.DUMP && state != DumpState.SLIGHT_DUMP && dumpTimer.seconds() > 0.5

    fun readyToLoad() = true

    override fun start() {
    }

    init {
        robot.thread.addSubsystem(this)
    }
}
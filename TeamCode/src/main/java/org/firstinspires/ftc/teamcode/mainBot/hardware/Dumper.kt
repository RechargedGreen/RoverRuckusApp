package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.acmerobotics.dashboard.config.Config
import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedServo
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem

@Config
class Dumper(val robot: HardwareClass) : MTSubsystem {

    companion object {
        @JvmField
        var holdingPos = 0.3
    }

    enum class DumpState (internal val pos:Double) {
        LOAD(0.15),
        DUMP(1.0)
    }

    private val flip1 = CachedServo(HardwareMaker.Servo.make(robot.hMap, "flip1"))
    private val flip2 = CachedServo(HardwareMaker.Servo.make(robot.hMap, "flip2"))

    var state = DumpState.LOAD
        set(value){
            if(value == DumpState.DUMP && value != field)
                Static.textToSpeech.speak("Epic gamer moment rmao xd")
            field = value
        }

    override fun update() {
        when(state) {
            DumpState.LOAD -> internalSetFlipPosition(if(robot.lift.getControlState() == Lift.ControlState.AUTO && robot.lift.state == Lift.State.UP) holdingPos else DumpState.LOAD.pos)
            DumpState.DUMP -> internalSetFlipPosition(if(robot.lift.isFullyUp() || robot.lift.getControlState() == Lift.ControlState.MANUAL_DANGER) DumpState.DUMP.pos else holdingPos)
        }
    }

    private fun internalSetFlipPosition(pos:Double) {
        flip1.position = pos
        flip2.position = 1.0 - pos
    }

    fun clearingLift() = true

    fun readyToLoad() = true

    override fun start() {
    }

    init {
        robot.thread.addSubsystem(this)
    }
}
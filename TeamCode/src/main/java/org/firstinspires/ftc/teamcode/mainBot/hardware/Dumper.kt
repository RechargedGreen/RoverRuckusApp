package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.acmerobotics.dashboard.config.Config
import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedServo
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.robotcore.util.ElapsedTime

@Config
class Dumper(val robot: HardwareClass) : MTSubsystem {

    enum class DumpState (internal val pos:Double) {
        LOAD(0.15),
        DUMP(1.0),
        HOLD(0.4)
    }

    private val flipL = CachedServo(HardwareMaker.Servo.make(robot.hMap, "flipL"))
    private val flipR = CachedServo(HardwareMaker.Servo.make(robot.hMap, "flipR"))

    private var lastState:DumpState? = null

    var state = DumpState.LOAD
        set(value){
            if(value == DumpState.DUMP && value != field)
                Static.textToSpeech.speak("Epic gamer moment rmao xd")
            else if(lastState == DumpState.DUMP && value != DumpState.DUMP)
                dumpTimer.reset()
            lastState = field
            field = value
        }

    val dumpTimer = ElapsedTime()

    override fun update() {
        when(state) {
            DumpState.LOAD -> internalSetFlipPosition(if(robot.lift.getControlState() != Lift.ControlState.MANUAL_DANGER && (robot.lift.state == Lift.State.UP || !robot.lift.isFullyDown())) DumpState.HOLD.pos else DumpState.LOAD.pos)
            DumpState.DUMP -> internalSetFlipPosition(if(robot.lift.isFullyUp() || robot.lift.getControlState() == Lift.ControlState.MANUAL_DANGER) DumpState.DUMP.pos else DumpState.HOLD.pos)
            DumpState.HOLD -> internalSetFlipPosition(DumpState.HOLD.pos)
        }
    }

    private fun internalSetFlipPosition(pos:Double) {
        flipL.position = pos
        flipR.position = 1.0 - pos
    }

    fun clearingLift() = clearingDown() && clearingUp()
    fun clearingUp() = true
    fun clearingDown() = dumpTimer.seconds() > 0.5

    fun readyToLoad() = true

    override fun start() {
    }

    init {
        robot.thread.addSubsystem(this)
    }
}
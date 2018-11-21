package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedServo
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem

class Dumper(val robot: HardwareClass) : MTSubsystem {

    enum class DumpState (internal val pos:Double) {
        LOAD(0.0),
        DUMP(1.0)
    }

    private val flip1 = CachedServo(HardwareMaker.Servo.make(robot.hMap, "flip1"))
    private val flip2 = CachedServo(HardwareMaker.Servo.make(robot.hMap, "flip2"))

    var state = DumpState.LOAD

    override fun update() {
        when(state) {
            DumpState.LOAD -> internalSetFlipPosition(DumpState.LOAD.pos)
            DumpState.DUMP -> internalSetFlipPosition((if(robot.lift.isFullyUp() || robot.lift.getControlState() == Lift.ControlState.MANUAL_DANGER) DumpState.DUMP else DumpState.LOAD).pos)
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
package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedServo
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem

class Dumper(val robot: HardwareClass) : MTSubsystem {

    enum class DumpState {
        HOLD,
        DUMP
    }

    private enum class InternalFlipState(val pos1: Double) {
        DUMP(0.0),
        LOAD(0.0)
    }

    private enum class InternalDumpState(val pos1: Double) {
        HOLD(0.0),
        DUMP(0.0)
    }

    //private val dump1 = CachedServo(HardwareMaker.Servo.make(robot.hMap, "dump1"))
    //private val flip1 = CachedServo(HardwareMaker.Servo.make(robot.hMap, "flip1"))

    var dumpState = DumpState.HOLD

    override fun update() {
        setInternalFlipState(if(robot.lift.isFullyUp() && robot.lift.state == Lift.State.UP) InternalFlipState.DUMP else InternalFlipState.LOAD)
        setInternalDumpState(when(dumpState){
            DumpState.HOLD -> InternalDumpState.HOLD
            DumpState.DUMP -> InternalDumpState.DUMP
        })
    }


    private fun setInternalFlipState(state: InternalFlipState) {
        //flip1.position = state.pos1
    }

    private fun setInternalDumpState(state: InternalDumpState) {
        //dump1.position = state.pos1
    }

    fun clearingLift() = true

    fun readyToLoad() = true

    override fun start() {
    }

    init {
        robot.thread.addSubsystem(this)
    }
}
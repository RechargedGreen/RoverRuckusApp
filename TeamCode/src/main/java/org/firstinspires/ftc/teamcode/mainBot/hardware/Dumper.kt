package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedServo
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem

class Dumper(val robot: HardwareClass) : MTSubsystem {
    enum class FlipState {
        FRONT,
        MID,
        BACK
    }

    enum class DumpState {
        HOLD,
        DUMP
    }

    private enum class InternalFlipState(val pos1: Double) {
        FRONT(0.0),
        BACK(0.0)
    }

    private enum class InternalDumpState(val pos1: Double) {
        HOLD(0.0),
        DUMP(0.0)
    }

    //private val dump1 = CachedServo(HardwareMaker.Servo.make(robot.hMap, "dump1"))
    //private val flip1 = CachedServo(HardwareMaker.Servo.make(robot.hMap, "flip1"))

    var flipState = FlipState.FRONT
    var dumpState = DumpState.HOLD

    override fun update() {
        when (flipState) {
            FlipState.FRONT -> setInternalFlipState(InternalFlipState.FRONT)
            FlipState.BACK  -> setInternalFlipState(InternalFlipState.BACK)
        }
        when (dumpState) {
            DumpState.DUMP -> setInternalDumpState(if (atFlipState()) InternalDumpState.DUMP else InternalDumpState.HOLD)
            DumpState.HOLD -> setInternalDumpState(InternalDumpState.HOLD)
        }
    }


    private fun setInternalFlipState(state: InternalFlipState) {
        //flip1.position = state.pos1
    }

    private fun setInternalDumpState(state: InternalDumpState) {
        //dump1.position = state.pos1
    }

    fun atFlipState(): Boolean = true

    override fun start() {
    }

    init {
        robot.thread.addSubsystem(this)
    }
}
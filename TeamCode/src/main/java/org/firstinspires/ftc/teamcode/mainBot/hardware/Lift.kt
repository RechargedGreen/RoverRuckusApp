package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedDcMotorEx
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.OptimumDigitalInput
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.RevTouchSensor
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.Range

class Lift(val robot: HardwareClass) : MTSubsystem {
    var dumpSafe = false
        private set

    private val liftStallSpeed = 0.2 // 0.1 was nice

    var state: State = State.DOWN
        set(value) {
            controlState = ControlState.AUTO
            field = value
        }

    @Throws(InterruptedException::class)
    fun deploy() {
        state = State.STOP
        robot.opMode.sleepSeconds(0.25)
        state = State.UP
        robot.opMode.waitTill { isFullyUp() }
    }

    private val downSensor = RevTouchSensor(OptimumDigitalInput(robot.getHub(1), 1))
    private val upSensor = RevTouchSensor(OptimumDigitalInput(robot.getHub(1), 7))

    private val motorL = CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "liftL"), robot.getHub(1))
    private val motorR = CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "liftR", DcMotorSimple.Direction.REVERSE), robot.getHub(1))

    private val latch = HardwareMaker.Servo.make(robot.hMap, "latch")

    @Throws(InterruptedException::class)
    private fun internalSetMotorPowers(power: Double, safe: Boolean) {
        val power = if (safe) Range.clip(power, if (isFullyDown() || !robot.dumper.clearingDown()) 0.0 else -1.0, if (!robot.dumper.clearingUp()) 0.0 else if(isFullyUp()) liftStallSpeed else 1.0) else power
        motorL.power = power
        motorR.power = power
    }

    private var openLoop = 0.0
    @Throws(InterruptedException::class)
    fun getControlState() = controlState

    enum class ControlState {
        MANUAL_SAFE,
        MANUAL_DANGER,
        AUTO
    }

    fun retract() {
        state = State.DOWN
        robot.dumper.state = Dumper.DumpState.LOAD
    }

    private var controlState = ControlState.AUTO
    @Throws(InterruptedException::class)
    fun setOpenLoopPower(power: Double, useFailSafes: Boolean = true) {
        openLoop = power
        controlState = if (useFailSafes) ControlState.MANUAL_SAFE else ControlState.MANUAL_DANGER
    }

    enum class State {
        DOWN,
        UP,
        LATCH_ENGAGED,
        STOP
    }

    @Throws(InterruptedException::class)
    override fun update() {
        if(isFullyUp() &&  state == State.UP)
            dumpSafe = true
        if(state != State.UP)
            dumpSafe = false

        setInternalLatchState(if (state == State.LATCH_ENGAGED) InternalLatchState.LATCHED else InternalLatchState.FREE)
        when (controlState) {
            ControlState.AUTO -> when (state) {
                State.UP -> setInternalState(if (isFullyUp()) InternalState.STOP else InternalState.GO_UP)
                State.DOWN -> setInternalState(if (isFullyDown() || !robot.dumper.clearingLift()) InternalState.STOP else InternalState.GO_DOWN)
                State.LATCH_ENGAGED, State.STOP -> setInternalState(InternalState.STOP)
            }
            ControlState.MANUAL_DANGER -> internalSetMotorPowers(openLoop, false)
            ControlState.MANUAL_SAFE -> internalSetMotorPowers(openLoop, true)
        }
    }

    @Throws(InterruptedException::class)
    private fun setInternalLatchState(state: InternalLatchState) {
        latch.position = state.pos
    }

    private enum class InternalLatchState(val pos: Double) {
        LATCHED(0.0),
        FREE(0.45)
    }

    private enum class InternalState(val power: Double) {
        GO_UP(1.0),
        STOP(0.0),
        GO_DOWN(-1.0)
    }

    private fun setInternalState(state: InternalState) = internalSetMotorPowers(state.power, true)
    @Throws(InterruptedException::class)
    fun isFullyDown(): Boolean = downSensor.pressed()

    @Throws(InterruptedException::class)
    fun isFullyUp(): Boolean = upSensor.pressed()

    @Throws(InterruptedException::class)
    override fun start() {
    }

    init {
        robot.thread.addSubsystem(this)
        state = if (robot.opMode.isAutonomous()) State.LATCH_ENGAGED else State.DOWN
    }
}
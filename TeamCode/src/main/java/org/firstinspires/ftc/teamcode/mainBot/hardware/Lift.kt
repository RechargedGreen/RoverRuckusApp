package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.Podoy_KW4_3Z_3_Micro_LimitSwitch
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.OptimumDigitalInput
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem

class Lift(val robot: HardwareClass) : MTSubsystem {
    private var state: State
        set(value) {
            controlState = ControlState.AUTO
            state = value
        }
        get() = state

    fun deploy(){
        state = State.UP
        robot.opMode.waitTill { isFullyUp() }
        state = State.DOWN
        robot.opMode.waitTill { isFullyDown() }
    }

    private val downSensor = Podoy_KW4_3Z_3_Micro_LimitSwitch(OptimumDigitalInput(robot.getHub(0), 0))
    private val upSensor = Podoy_KW4_3Z_3_Micro_LimitSwitch(OptimumDigitalInput(robot.getHub(0), 1))

    private val motorL = robot.hMap.dcMotor.get("liftL")
    private val motorR = robot.hMap.dcMotor.get("liftR")

    private fun internalSetMotorPowers(power: Double) {
        motorL.power = power
        motorR.power = power
    }

    private var openLoop = 0.0

    private enum class ControlState {
        MANUAL_SAFE,
        MANUAL_DANGER,
        AUTO
    }

    private var controlState = ControlState.AUTO

    private fun setOpenLoopPower(power: Double, useFailSafes: Boolean = true) {
        openLoop = power
        controlState = if (useFailSafes) ControlState.MANUAL_SAFE else ControlState.MANUAL_DANGER
    }

    enum class State {
        DOWN,
        UP
    }

    override fun update() {
        when(controlState) {
            ControlState.AUTO -> when(state){
                State.UP -> setInternalState(if(isFullyUp()) InternalState.STOP else InternalState.GO_UP)
                State.DOWN -> setInternalState(if(isFullyDown()) InternalState.STOP else InternalState.GO_DOWN)
            }
            ControlState.MANUAL_DANGER -> internalSetMotorPowers(openLoop)
            ControlState.MANUAL_SAFE -> internalSetMotorPowers(if(isFullyDown() || isFullyUp()) InternalState.STOP.power else openLoop)
        }
    }

    private enum class InternalState(val power:Double){
        GO_UP(1.0),
        STOP(0.0),
        GO_DOWN(-1.0)
    }
    private fun setInternalState(state:InternalState) = internalSetMotorPowers(state.power)

    fun isFullyDown():Boolean = downSensor.pressed()
    fun isFullyUp():Boolean = upSensor.pressed()

    override fun start() {
    }

    init {
        robot.thread.addSubsystem(this)
        state = State.DOWN
    }
}
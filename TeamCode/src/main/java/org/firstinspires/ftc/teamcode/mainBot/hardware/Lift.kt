package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedDcMotorEx
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.Podoy_KW4_3Z_3_Micro_LimitSwitch
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.OptimumDigitalInput
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.RevTouchSensor
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.Range

class Lift(val robot: HardwareClass) : MTSubsystem {
    var state: State = State.DOWN
        set(value) {
            controlState = ControlState.AUTO
            field = value
        }

    fun deploy(){
        state = State.UP
        robot.opMode.waitTill { isFullyUp() }
        state = State.DOWN
        robot.opMode.waitTill { isFullyDown() }
    }

    private val downSensor = RevTouchSensor(OptimumDigitalInput(robot.getHub(0), 3))// channel 2
    private val upSensor = Podoy_KW4_3Z_3_Micro_LimitSwitch(OptimumDigitalInput(robot.getHub(0), 1))// channel 1

    private val motorL = CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "liftL"), robot.getHub(1))
    private val motorR = CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "liftR", DcMotorSimple.Direction.REVERSE), robot.getHub(1))

    private fun internalSetMotorPowers(power: Double, safe:Boolean) {
        val power = if(safe) Range.clip(power, if(isFullyDown()) 0.0 else -1.0, if(isFullyUp()) 0.0 else 1.0) else power
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

    fun setOpenLoopPower(power: Double, useFailSafes: Boolean = true) {
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
                State.DOWN -> setInternalState(if(isFullyDown() || !robot.dumper.clearingLift()) InternalState.STOP else InternalState.GO_DOWN)
            }
            ControlState.MANUAL_DANGER -> internalSetMotorPowers(openLoop, false)
            ControlState.MANUAL_SAFE -> internalSetMotorPowers(openLoop, true)
        }
    }

    private enum class InternalState(val power:Double){
        GO_UP(1.0),
        STOP(0.0),
        GO_DOWN(-1.0)
    }
    private fun setInternalState(state:InternalState) = internalSetMotorPowers(state.power, true)

    fun isFullyDown():Boolean = downSensor.pressed()
    fun isFullyUp():Boolean = upSensor.pressed()

    override fun start() {
    }

    init {
        robot.thread.addSubsystem(this)
        state = State.DOWN
    }
}
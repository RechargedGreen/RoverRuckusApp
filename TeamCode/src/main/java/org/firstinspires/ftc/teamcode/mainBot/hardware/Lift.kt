package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.RevTouchSensor
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
        robot.opMode.loopWhile(action = {}, condition = {!isFullyUp()})
        state = State.DOWN
        robot.opMode.loopWhile(action = {}, condition = {!isFullyDown()})
    }

    private val downSensor = RevTouchSensor(robot.getHub(0), 0)
    private val upSensor = RevTouchSensor(robot.getHub(0), 0)

    //private val motor1 = robot.hMap.dcMotor.get("lift1")
    //private val motor2 = robot.hMap.dcMotor.get("lift1")

    private fun internalSetMotorPowers(power: Double) {
        //motor1.power = power
        //motor2.power = power
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
    }

    fun isFullyDown():Boolean = downSensor.pressed()
    fun isFullyUp():Boolean = upSensor.pressed()

    override fun start() {
    }

    init {
        robot.thread.addSubsystem(this)
        state = State.DOWN
    }
}
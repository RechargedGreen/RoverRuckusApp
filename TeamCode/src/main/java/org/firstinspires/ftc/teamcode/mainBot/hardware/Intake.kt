package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import org.firstinspires.ftc.teamcode.MineralType

class Intake(val robot: HardwareClass) : MTSubsystem {
    enum class State {
        STOPPED,
        BLIND_IN,
        GOLD_IN,
        SILVER_IN,
        OUT
    }

    private enum class InternalState(val power: Double) {
        IN(1.0),
        OUT(-1.0),
        STOP(0.0)
    }

    var state = State.STOPPED

    private val delegate = robot.hMap.dcMotor.get("intake")

    override fun update() {
        when (state) {
            State.STOPPED   -> setInternalState(InternalState.STOP)
            State.BLIND_IN  -> setInternalState(InternalState.IN)
            State.OUT       -> setInternalState(InternalState.OUT)
            State.GOLD_IN   -> setInternalState(if (currentlyInIntake() == MineralType.SILVER) InternalState.OUT else InternalState.IN)
            State.SILVER_IN -> setInternalState(if (currentlyInIntake() == MineralType.GOLD) InternalState.OUT else InternalState.IN)
        }
    }

    private fun currentlyInIntake(): MineralType = MineralType.UNKNOWN

    private fun setInternalState(state: InternalState) {
        delegate.power = state.power
    }

    override fun start() {
    }

    init {
        robot.thread.addSubsystem(this)
    }
}
package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.hardware.rev.RevBlinkinLedDriver

class SuperSystem(val robot: HardwareClass) : MTSubsystem {
    val blinken = robot.hMap.get(RevBlinkinLedDriver::class.java, "blinken") //as CachedBlinkenLED
    val normalPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN
    val hangTimePattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED

    enum class State {
        RESET
    }

    fun setState(state: State) {
        when (state) {
            State.RESET -> {
                robot.intake.intakeState = Intake.IntakeState.STOP
                robot.intake.extensionState = Intake.IntakeExtensionState.IN
                robot.intake.sortState = Intake.SortState.BLIND
                robot.dumper.state = Dumper.DumpState.LOAD
                robot.lift.state = Lift.State.DOWN
            }
        }
    }

    override fun update() {
        if(robot.opMode.isAutonomous())
            blinken.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK)
        else
            blinken.setPattern(if (120.0 - robot.opMode.runtime.seconds() < 10.0) hangTimePattern else normalPattern)
    }
    override fun start() {}

    init {
        robot.thread.addSubsystem(this)
    }
}
package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.hardware.rev.RevBlinkinLedDriver

class SuperSystem(val robot: HardwareClass) : MTSubsystem {
    val blinken = robot.hMap.get(RevBlinkinLedDriver::class.java, "blinken")
    val normalPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN
    val hangTimePattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED

    enum class State {
        RESET
    }

    fun setState(state: State) {
        when (state) {
            State.RESET -> {
                robot.intake.state = Intake.State.STOPPED
                robot.dumper.dumpState = Dumper.DumpState.HOLD
                robot.lift.state = Lift.State.DOWN
            }
        }
    }

    override fun update() = blinken.setPattern(if (120.0 - robot.opMode.runtime.seconds() < 10.0) hangTimePattern else normalPattern)
    override fun start() {}

    enum class SampleSituation{
        LANDER_SILVER,
        LANDER_GOLD,
        DEPOT_GOLD
    }

    fun sample(situation:SampleSituation){
        when(situation){
            SampleSituation.LANDER_SILVER -> {

            }
            SampleSituation.LANDER_GOLD -> {

            }
            SampleSituation.DEPOT_GOLD -> {

            }
        }
    }
}
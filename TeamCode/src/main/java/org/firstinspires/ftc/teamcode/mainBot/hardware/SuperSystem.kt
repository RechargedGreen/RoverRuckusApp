package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.util.ElapsedTime

class SuperSystem(val robot: HardwareClass) : MTSubsystem {
    val blinken = robot.hMap.get(RevBlinkinLedDriver::class.java, "blinken") //as CachedBlinkenLED
    val normalPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN
    val hangTimePattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED

    enum class State {
        UNKNOWN,
        LOWER_LIFT_AFTER_MARKER
    }

    val timeAfterStateChange = ElapsedTime()

    var state = State.UNKNOWN
        set(value){
            timeAfterStateChange.reset()
            field = value
        }

    override fun update() {
        when(state){
            State.LOWER_LIFT_AFTER_MARKER -> {
                robot.dumper.state = Dumper.DumpState.LOAD
                if(timeAfterStateChange.seconds() > 2.0){
                    robot.lift.state = Lift.State.DOWN
                    state = State.UNKNOWN
                }
            }
            State.UNKNOWN ->{}
        }
        if(robot.opMode.isAutonomous() && robot.opMode.isStarted){
            robot.opMode.telemetry.addData("status", "running auto")
            robot.opMode.telemetry.update()
        }
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
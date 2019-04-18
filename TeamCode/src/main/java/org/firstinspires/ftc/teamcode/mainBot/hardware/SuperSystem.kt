package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.util.ElapsedTime

class SuperSystem(val robot: HardwareClass) : MTSubsystem {
    val blinken = robot.hMap.get(RevBlinkinLedDriver::class.java, "blinken") //as CachedBlinkenLED
    val normalPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN
    val hangTimePattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED

    val bucketSense = BucketSense(robot.hMap)

    enum class State {
        UNKNOWN,
        FOLLOWING_FSM
    }

    val timeAfterStateChange = ElapsedTime()

    var fsm:SuperStructureFSM? = null
        set(value){
            field = value
            state = State.FOLLOWING_FSM
        }

    fun waitOnFSM() = robot.opMode.waitWhile { isFollowingFSM() }

    var state = State.UNKNOWN
        set(value) {
            timeAfterStateChange.reset()
            field = value
        }

    @Throws(InterruptedException::class)
    override fun update() {
        if (!robot.opMode.isAutonomous())
            bucketSense.updateCache()
        when (state) {
            State.UNKNOWN -> {
            }
            State.FOLLOWING_FSM -> {
                val fsm = fsm
                if(fsm == null || fsm.isComplete())
                    state = State.UNKNOWN
                else
                    fsm.update()
            }
        }
        if (robot.opMode.isAutonomous() && robot.opMode.isStarted) {
            robot.opMode.telemetry.addData("status", "running auto")
            robot.opMode.telemetry.addLine()
            robot.opMode.telemetry.update()
        }
        if (robot.opMode.isAutonomous())
            blinken.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK)
        else
            blinken.setPattern(if (120.0 - robot.opMode.runtime.seconds() < 10.0) hangTimePattern else normalPattern)
    }

    @Throws(InterruptedException::class)
    override fun start() {
    }

    init {
        robot.thread.addSubsystem(this)
    }

    fun isFollowingFSM() = state == State.FOLLOWING_FSM
}

abstract class SuperStructureFSM(protected val robot:HardwareClass){
    abstract fun update()
    abstract fun isComplete():Boolean
}
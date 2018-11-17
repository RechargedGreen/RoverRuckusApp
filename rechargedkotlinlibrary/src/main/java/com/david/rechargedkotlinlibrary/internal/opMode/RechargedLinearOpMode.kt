package com.david.rechargedkotlinlibrary.internal.opMode

import com.acmerobotics.dashboard.FtcDashboard
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime

/**
 * Created by David Lukens on 8/2/2018.
 */
abstract class RechargedLinearOpMode<rt : RobotTemplate>(private val autonomous: Boolean, var alliance: Alliance = Alliance.FLUID, private val createRobot: (RechargedLinearOpMode<rt>) -> rt) : LinearOpMode() {
    lateinit var robot: rt
    fun isAutonomous(): Boolean = autonomous

    val runtime: ElapsedTime = ElapsedTime()

    lateinit var dash:FtcDashboard

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        dash = FtcDashboard.getInstance()
        robot = createRobot(this)
        if (autonomous)
            robot.start()
        if (autonomous)
            robot.autoPostInit()
        while(!isStarted && !isStopRequested)
            tillStart()
        telemetry.addData("status", "started")
        if (autonomous)
            robot.onPressingAutoPlay()
        else {
            robot.start()
            robot.onPressingTeleOpPlay()
        }

        runtime.reset()
        run()
    }

    @Throws(InterruptedException::class)
    abstract fun run()

    fun loop(action: () -> Unit) {
        loopWhile({ true }, action)
    }

    fun loopWhile(condition: () -> Boolean = {true}, action: () -> Unit = {}) {
        while (condition() && opModeIsActive())
            action()
    }

    fun waitWhile(condition: () -> Boolean) = loopWhile(condition = condition)

    fun loopTill(condition: () -> Boolean = {true}, action: () -> Unit = {}){
        while (!condition() && opModeIsActive())
            action()
    }

    fun sleepSeconds(seconds:Double) = sleep((seconds * 1000).toLong())

    fun waitTill(condition: () -> Boolean) = loopTill(condition)

    open fun tillStart() = preventTimeOut()

    private fun preventTimeOut() {
        telemetry.addData("Status", "Waiting in Init")
        telemetry.update()
    }
}
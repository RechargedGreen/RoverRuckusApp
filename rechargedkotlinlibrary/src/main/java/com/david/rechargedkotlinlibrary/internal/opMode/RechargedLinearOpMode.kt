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

    lateinit var dash: FtcDashboard

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        dash = FtcDashboard.getInstance()
        robot = createRobot(this)
        if (autonomous) {
            robot.start()
            robot.autoPostInit()
        }
        while (!isStarted && !isStopRequested)
            tillStart()
        if(opModeIsActive()) {
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
    }

    @Throws(InterruptedException::class)
    abstract fun run()

    @Throws(InterruptedException::class)
    fun loop(action: () -> Unit) {
        loopWhile({ true }, action)
    }

    @Throws(InterruptedException::class)
    fun loopWhile(condition: () -> Boolean = { true }, action: () -> Unit = {}) {
        while (condition() && opModeIsActive())
            action()
    }

    @Throws(InterruptedException::class)
    fun sleepTillTime(seconds: Double) = waitWhile { runtime.seconds() < seconds }
    @Throws(InterruptedException::class)
    fun waitWhile(condition: () -> Boolean) = loopWhile(condition = condition)
    @Throws(InterruptedException::class)
    fun loopTill(condition: () -> Boolean = { true }, action: () -> Unit = {}) {
        while (!condition() && opModeIsActive())
            action()
    }
    @Throws(InterruptedException::class)
    fun sleepSeconds(seconds: Double) = sleep((seconds * 1000).toLong())
    @Throws(InterruptedException::class)
    fun waitTill(condition: () -> Boolean) = loopTill(condition)
    @Throws(InterruptedException::class)
    open fun tillStart() = preventTimeOut()
    @Throws(InterruptedException::class)
    private fun preventTimeOut() {
        telemetry.addData("Status", "Waiting in Init")
        telemetry.update()
    }
}
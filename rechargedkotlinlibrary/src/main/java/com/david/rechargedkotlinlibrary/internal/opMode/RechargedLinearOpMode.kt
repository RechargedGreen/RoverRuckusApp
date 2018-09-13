package com.david.rechargedkotlinlibrary.internal.opMode

import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.qualcomm.robotcore.util.ElapsedTime
import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode

/**
 * Created by David Lukens on 8/2/2018.
 */
abstract class RechargedLinearOpMode<rt : RobotTemplate>(private val autonomous: Boolean, var alliance: Alliance = Alliance.FLUID, private val createRobot: (RechargedLinearOpMode<rt>) -> rt) : TunableLinearOpMode() {
    lateinit var robot: rt
    fun isAutonomous(): Boolean = autonomous

    val runtime: ElapsedTime = ElapsedTime()

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        try {
            robot = createRobot(this)
            if (autonomous)
                robot.start()
            if (autonomous)
                robot.autoPostInit()
            waitForStart()
            if (!autonomous)
                robot.start()
            runtime.reset()
            run()
        } catch (e: InterruptedException) {
            throw e
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    @Throws(InterruptedException::class)
    abstract fun run()

    fun loop(action: () -> Unit) {
        loopWhile({ true }, action)
    }

    fun loopWhile(condition: () -> Boolean, action: () -> Unit) {
        while (condition().and(opModeIsActive()))
            action()
    }
}
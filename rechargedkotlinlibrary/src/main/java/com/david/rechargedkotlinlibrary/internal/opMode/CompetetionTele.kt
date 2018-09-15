package com.david.rechargedkotlinlibrary.internal.opMode

import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate

/**
 * Created by David Lukens on 8/2/2018.
 */

abstract class CompetetionTele<rt : RobotTemplate>(createRobot: (RechargedLinearOpMode<rt>) -> rt) : RechargedLinearOpMode<rt>(false, Alliance.FLUID, createRobot) {
    var practice = false
    lateinit var c1: SimpleController
    lateinit var c2: SimpleController

    @Throws(InterruptedException::class)
    override fun run() {
        onStart()
        loopWhile({ practice || runtime.seconds() < 120.0 }, {
            c1 = SimpleController(gamepad1)
            c2 = SimpleController(gamepad2)
            onLoop()
            telemetry.update()
        })
    }

    @Throws(InterruptedException::class)
    open fun onStart() {
    }

    @Throws(InterruptedException::class)
    abstract fun onLoop()
}
package com.david.rechargedkotlinlibrary.internal.opMode

import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate

abstract class PracticeTeleOp<rt : RobotTemplate>(createRobot: (RechargedLinearOpMode<rt>) -> rt) : CompetetionTele<rt>(createRobot) {
    @Throws(InterruptedException::class)
    override fun run() {
        practice = true
        super.run()
    }
}
package com.david.rechargedkotlinlibrary.internal.opMode

import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate

/**
 * Created by David Lukens on 8/2/2018.
 */

abstract class FluidAuto<rt : RobotTemplate>(createRobot: (RechargedLinearOpMode<rt>) -> rt) : RechargedLinearOpMode<rt>(true, Alliance.FLUID, createRobot)
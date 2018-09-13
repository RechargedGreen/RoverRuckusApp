package com.david.rechargedkotlinlibrary.internal.hardware.management

/**
 * Created by David Lukens on 8/2/2018.
 */

abstract class SameThreadSubsystem(robot: RobotTemplate) : Subsystem(robot) {
    init {
        robot.sameThreadSubsystems.add(this)
    }
}
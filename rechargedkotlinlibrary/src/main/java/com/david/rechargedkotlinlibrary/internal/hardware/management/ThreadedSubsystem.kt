package com.david.rechargedkotlinlibrary.internal.hardware.management

/**
 * Created by David Lukens on 8/2/2018.
 */

abstract class ThreadedSubsystem(robot: RobotTemplate) : Subsystem(robot), MTSubsystem {
    init {
        robot.thread.addSubsystem(this)
    }
    @Throws(InterruptedException::class)
    override fun start() {}
}
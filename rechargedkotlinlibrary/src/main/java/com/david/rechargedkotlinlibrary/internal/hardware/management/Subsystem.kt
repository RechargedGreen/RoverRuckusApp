package com.david.rechargedkotlinlibrary.internal.hardware.management

/**
 * Created by David Lukens on 8/2/2018.
 */
abstract class Subsystem(robot: RobotTemplate) : SubSystemBase {
    protected val hMap = robot.hMap
}
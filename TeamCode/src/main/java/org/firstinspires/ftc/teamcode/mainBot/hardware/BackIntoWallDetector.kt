package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.acmerobotics.dashboard.config.Config
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem

@Config
class BackIntoWallDetector(robot: HardwareClass) : MTSubsystem {

    companion object

    var threshold = 0

    //private val sensor = robot.hMap.get(ModernRoboticsI2cRangeSensor::class.java, "backIntoWallUltrasonic")

    var lastKnownDistance: Double? = null
    var enabled = false

    @Throws(InterruptedException::class)
    fun close(): Boolean {
        val cache = lastKnownDistance
        return cache != null && cache < threshold
    }

    @Throws(InterruptedException::class)
    fun far() = !close()

    @Throws(InterruptedException::class)
    override fun update() {
        //  lastKnownDistance = if (enabled) sensor.cmUltrasonic() / 2.54 else null
    }

    @Throws(InterruptedException::class)
    override fun start() {}

    init {
        robot.thread.addSubsystem(this)
    }
}
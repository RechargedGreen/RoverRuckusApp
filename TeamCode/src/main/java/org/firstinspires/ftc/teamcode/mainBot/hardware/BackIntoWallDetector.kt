package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.acmerobotics.dashboard.config.Config
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor

@Config
class BackIntoWallDetector(robot: HardwareClass) : MTSubsystem {

    companion object

    var threshold = 0

    private val sensor = robot.hMap.get(ModernRoboticsI2cRangeSensor::class.java, "backIntoWallUltrasonic")

    var lastKnownDistance: Double? = null
    var enabled = false

    fun close(): Boolean {
        val cache = lastKnownDistance
        return cache != null && cache < threshold
    }

    fun far() = !close()

    override fun update() {
        lastKnownDistance = if (enabled) sensor.cmUltrasonic() / 2.54 else null
    }

    override fun start() {}

    init {
        robot.thread.addSubsystem(this)
    }
}
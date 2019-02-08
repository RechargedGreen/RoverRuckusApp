package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.acmerobotics.dashboard.config.Config
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@Config
class CraterRimDetector(robot: HardwareClass) : MTSubsystem {
    companion object {
        @JvmField
        var closeThreshold = 0.0
        @JvmField
        var mediumThreshold = 0.0
    }

    private val sensor = robot.opMode.hardwareMap.get(DistanceSensor::class.java, "craterRimDetector") as Rev2mDistanceSensor

    private var distance: Double? = null
    var enabled = true

    fun checkThreshold(threshold: Double): Boolean {
        val cache = distance
        return if (cache != null)
            cache < threshold
        else
            false
    }

    fun close(): Boolean = checkThreshold(closeThreshold)
    fun medium() = checkThreshold(mediumThreshold)
    fun far() = !(medium() || close())

    fun getDistance() = distance

    override fun update() {
        distance = if (enabled) sensor.getDistance(DistanceUnit.INCH) else null
    }

    override fun start() {
    }

    init {
        robot.thread.addSubsystem(this)
    }
}
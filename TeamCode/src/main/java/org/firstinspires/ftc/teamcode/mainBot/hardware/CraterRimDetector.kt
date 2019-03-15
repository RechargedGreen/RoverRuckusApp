package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.acmerobotics.dashboard.config.Config
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem

@Config
class CraterRimDetector(robot: HardwareClass) : MTSubsystem {
    companion object {
        @JvmField
        var closeThreshold = 0.0
        @JvmField
        var mediumThreshold = 0.0
    }

    //private val sensor = robot.opMode.hardwareMap.get(DistanceSensor::class.java, "craterRimDetector") as Rev2mDistanceSensor

    private var distance: Double? = null
    var enabled = true

    @Throws(InterruptedException::class)
    fun checkThreshold(threshold: Double): Boolean {
        val cache = distance
        return if (cache != null)
            cache < threshold
        else
            false
    }

    @Throws(InterruptedException::class)
    fun close(): Boolean = checkThreshold(closeThreshold)

    @Throws(InterruptedException::class)
    fun medium() = checkThreshold(mediumThreshold)

    @Throws(InterruptedException::class)
    fun far() = !(medium() || close())

    @Throws(InterruptedException::class)
    fun getDistance() = distance

    @Throws(InterruptedException::class)
    override fun update() {
        //  distance = if (enabled) sensor.getDistance(DistanceUnit.INCH) else null
    }

    @Throws(InterruptedException::class)
    override fun start() {
    }

    init {
        robot.thread.addSubsystem(this)
    }
}
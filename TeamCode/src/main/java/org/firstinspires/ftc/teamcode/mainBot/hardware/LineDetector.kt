package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.robotcore.hardware.ColorSensor

class LineDetector(robot: HardwareClass) : MTSubsystem {

    var onLine = false
    var hasHit = false

    val reds = intArrayOf(0, 0)
    val blues = intArrayOf(0, 0)

    @Throws(InterruptedException::class)
    fun reset() {
        hasHit = false
    }

    private val redThreshold = 65
    private val blueThreshold = 60

    private val sensor = robot.hMap.get(ColorSensor::class.java, "lineSensor")

    @Throws(InterruptedException::class)
    override fun update() {
        if (enabled) {
            onLine = check(sensor, 1)
            if (onLine)
                hasHit = true
        }
    }

    var enabled = false
        set(value) {
            field = value
            if (value)
                reset()
        }

    @Throws(InterruptedException::class)
    private fun check(sensor: ColorSensor, index: Int): Boolean {
        val red = sensor.red()
        val blue = sensor.blue()
        reds[index] = red
        blues[index] = blue
        return if (index < reds.size && index < blues.size) red > redThreshold || blue > blueThreshold else false
    }

    @Throws(InterruptedException::class)
    override fun start() {
    }

    init {
        robot.thread.addSubsystem(this)
    }
}
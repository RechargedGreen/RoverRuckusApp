package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.robotcore.hardware.ColorSensor

class LineDetector(robot: HardwareClass) : MTSubsystem {

    var onLine = false
    var hasHit = false

    val reds = intArrayOf(0, 0)
    val blues = intArrayOf(0, 0)

    fun reset() {
        hasHit = false
    }

    private val redThreshold = 65
    private val blueThreshold = 60

    private val colorFront = /*LynxOptimizedI2cFactory.createLynxI2cColorRangeSensor(robot.getHub(0).delegate, 0)*/ robot.hMap.get(ColorSensor::class.java, "colorFront")
    private val colorBack = /*LynxOptimizedI2cFactory.createLynxI2cColorRangeSensor(robot.getHub(0).delegate, 1)*/ robot.hMap.get(ColorSensor::class.java, "colorBack")

    override fun update() {
        if (enabled) {
            onLine = check(colorFront, 0) || check(colorBack, 1)
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

    private fun check(sensor: ColorSensor, index: Int): Boolean {
        val red = sensor.red()
        val blue = sensor.blue()
        reds[index] = red
        blues[index] = blue
        return if (index < reds.size && index < blues.size) red > redThreshold || blue > blueThreshold else false
    }

    override fun start() {
    }

    init {
        robot.thread.addSubsystem(this)
    }
}
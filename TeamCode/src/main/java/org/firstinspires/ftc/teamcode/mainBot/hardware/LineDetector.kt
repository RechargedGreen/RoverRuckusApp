package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.robotcore.hardware.ColorSensor

class LineDetector (robot:HardwareClass):MTSubsystem{

    var onLine = false
    var hasHit = false

    val reds = intArrayOf(2)
    val blues = intArrayOf(2)

    fun reset(){
        hasHit = false
    }

    private val redThreshold = 100
    private val blueThreshold = 60

    private val colorFront:ColorSensor = robot.hMap.get(ColorSensor::class.java, "colorFront")
    private val colorBack:ColorSensor = robot.hMap.get(ColorSensor::class.java, "colorBack")

    override fun update() {
        onLine = check(colorFront, 0) || check(colorBack, 1)
        if(onLine)
            hasHit = true
    }

    private fun check(sensor:ColorSensor, index:Int) : Boolean{
        val red = sensor.red()
        val blue = sensor.blue()
        reds[index] = red
        blues[index] = blue
        return red > redThreshold || blue > blueThreshold
    }

    override fun start() {
    }

    init {
        robot.thread.addSubsystem(this)
    }
}
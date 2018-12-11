package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.robotcore.hardware.ColorSensor

class LineDetector (robot:HardwareClass):MTSubsystem{

    var r = 0
    var g = 0
    var b = 0
    var alpha = 0
    var hue = 0

    var onLine = false
    var hasHit = false

    fun reset(){
        hasHit = false
    }

    private val rThreshold = 50
    private val bThreshold = 40

    val color:ColorSensor = robot.hMap.get(ColorSensor::class.java, "color")

    override fun update() {
        r = color.red()
        //g = color.green()
        b = color.blue()
        //alpha = color.alpha()
        //hue = color.argb()
        onLine = r > rThreshold || b > bThreshold
        if(onLine)
            hasHit = true
    }

    override fun start() {
    }

    init {
        robot.thread.addSubsystem(this)
    }
}
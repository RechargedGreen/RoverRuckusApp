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

    private val threshold = 0

    val color:ColorSensor = robot.hMap.get(ColorSensor::class.java, "color")

    override fun update() {
        //r = color.red()
        //g = color.green()
        //b = color.blue()
        //alpha = color.alpha()
        hue = color.argb()
        onLine = hue > threshold
    }

    override fun start() {
    }

    init {
        robot.thread.addSubsystem(this)
    }
}
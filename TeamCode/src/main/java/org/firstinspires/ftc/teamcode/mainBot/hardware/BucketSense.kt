package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap

@Config
class BucketSense(hMap: HardwareMap) {

    companion object {
        @JvmField
        var threshold = 100000000 // 50000 was old bucket
    }

    private val left = hMap.colorSensor.get("leftBucketColor")
    private val right = hMap.colorSensor.get("rightBucketColor")

    var leftRed = 0
    var leftGreen = 0
    var leftBlue = 0
    var leftAlpha = 0
    var leftARGB = 0

    var rightRed = 0
    var rightGreen = 0
    var rightBlue = 0
    var rightAlpha = 0
    var rightARGB = 0

    @Throws(InterruptedException::class)
    fun updateCache() {
        //leftRed = left.red()
        //leftGreen = left.green()
        //leftBlue = left.blue()
        leftAlpha = left.alpha()
        leftARGB = left.argb()

        //rightRed = right.red()
        //rightGreen = right.green()
        //rightBlue = right.blue()
        rightAlpha = right.alpha()
        rightARGB = right.argb()
    }

    @Throws(InterruptedException::class)
    fun left() = leftARGB > threshold

    @Throws(InterruptedException::class)
    fun right() = rightARGB > threshold
}
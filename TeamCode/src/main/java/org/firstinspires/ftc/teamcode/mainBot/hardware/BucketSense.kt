package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@Config
class BucketSense(hMap: HardwareMap) {
    companion object {
        @JvmField
        var leftThreshold = 6.0
        @JvmField
        var rightThreshold = 6.0
    }

    private val leftDistanceSensor = hMap.get(DistanceSensor::class.java, "leftBucketColor")
    private val rightDistanceSensor = hMap.get(DistanceSensor::class.java, "rightBucketColor")
    //private val left = hMap.colorSensor.get("leftBucketColor")
    //private val right = hMap.colorSensor.get("rightBucketColor")

    var leftDistance = 10000.0
    var rightDistance = 10000.0

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

        //leftAlpha = left.alpha()
        //eftARGB = left.argb()

        //rightRed = right.red()
        //rightGreen = right.green()
        //rightBlue = right.blue()

        //rightAlpha = right.alpha()
        //rightARGB = right.argb()

        leftDistance = leftDistanceSensor.getDistance(DistanceUnit.CM)
        rightDistance = rightDistanceSensor.getDistance(DistanceUnit.CM)
    }

    @Throws(InterruptedException::class)
    fun left() = leftDistance < leftThreshold

    @Throws(InterruptedException::class)
    fun right() = rightDistance < rightThreshold

    /*@Throws(InterruptedException::class)
    fun left() = leftARGB > threshold

    @Throws(InterruptedException::class)
    fun right() = rightARGB > threshold*/
}

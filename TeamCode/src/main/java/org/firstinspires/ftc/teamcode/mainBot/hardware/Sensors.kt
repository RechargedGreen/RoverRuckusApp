package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.absoluteValue
import kotlin.math.cos

class Sensors(val robot:HardwareClass) : MTSubsystem{
    val lineDetector = LineDetector(robot)

    private val rightRange = robot.hMap.get(DistanceSensor::class.java, "rightRange") as Rev2mDistanceSensor

    private val rightRangeOffset = 7.5//todo get actual number
    private var rightRangeRaw = 0.0

    override fun update() {
        //rightRangeRaw = rightRange.getDistance(DistanceUnit.INCH)
    }
    override fun start() {
    }

    fun getRightDistanceFromWall(heading:Double) = adjustForHeading(heading, rightRangeRaw + rightRangeOffset)

    private fun adjustForHeading(heading:Double, distance:Double) = cos((robot.drive.imu.getZ(AngleUnit.RADIANS) - Math.toRadians(heading)).absoluteValue) * distance

    init {
        robot.thread.addSubsystem(this)
    }
}
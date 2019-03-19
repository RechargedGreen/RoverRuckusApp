package org.firstinspires.ftc.teamcode.mainBot.auto

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions
import kotlin.math.absoluteValue

@Config
object WorldsSilverSample {
    @JvmField
    var centerHitTicks = 0
    @JvmField
    var centerBackTicks = 0


    @JvmField
    var sideOffset = 0.0
    @JvmField
    var sideStartTicks = 0
    @JvmField
    var sideHitTicks = 0
    @JvmField
    var sideBackTicks = 0

    @JvmField
    var toWallTicks = 0

    private lateinit var opMode: RR2Auto
    private val SPEED = DriveTerrain.AngleFollowSpeeds.SLOW

    @Throws(InterruptedException::class)
    private fun drive(ticks: Int, angle: Double) = opMode.robot.drive.deadReckonPID(ticks, angle, SPEED)

    @Throws(InterruptedException::class)
    private fun turn(angle: Double) = opMode.robot.drive.pidTurn(angle)

    @Throws(InterruptedException::class)
    fun doStuff(opMode: RR2Auto) {
        this.opMode = opMode
        sample()
        toWall()
    }

    @Throws(InterruptedException::class)
    private fun sample() {
        val start = RR2Auto.StartingPositions.SILVER_HANG.angle
        when (opMode.ORDER) {
            SampleRandomizedPositions.LEFT -> {
                val angle = start + sideOffset
                drive(sideStartTicks, start)
                turn(angle)
                drive(sideHitTicks, angle)
                drive(-sideBackTicks, angle)
            }
            SampleRandomizedPositions.CENTER -> {
                drive(centerHitTicks, start)
                drive(centerBackTicks, start)
            }

            SampleRandomizedPositions.RIGHT -> {
            }
        }
    }

    @Throws(InterruptedException::class)
    private fun toWall() {
        val angle = RR2Auto.CompassDirection.NORTH_EAST.degrees
        turn(angle.absoluteValue)
        drive(toWallTicks, angle)
    }
}
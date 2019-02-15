package org.firstinspires.ftc.teamcode.iterative.testing.bot

import com.david.rechargedkotlinlibrary.internal.util.MathUtil
import com.qualcomm.robotcore.hardware.DcMotorSimple

/**
 * Created by David Lukens on 8/8/2018.
 */
open class IterativeEncoder(private val HUB: IterativeRevHub, private val PORT: Int, private val PPR: Int, direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD) {
    private var resetTicks = 0
    private var secant = 1

    init {
        setDirection(direction)
        reset()
    }

    fun getRawTicks() = HUB.getEncoder(PORT) * secant
    fun reset() {
        resetTicks = getRawTicks()
    }

    fun getTicks() = getRawTicks() - resetTicks
    fun getRadians() = toRadians(getTicks())
    fun getRawRadians() = toRadians(getRawTicks())

    fun toRadians(ticks: Int): Double {
        return (ticks.toDouble() / PPR.toDouble()) * MathUtil.TAU
    }

    fun setDirection(direction: DcMotorSimple.Direction) {
        secant = when (direction) {
            DcMotorSimple.Direction.REVERSE -> 1
            DcMotorSimple.Direction.FORWARD -> -1
        }
    }

    private var lastTicks: Int? = null

    fun tickChange(): Int? {
        var ticks = getTicks()
        var change: Int? = if (lastTicks != null) ticks - lastTicks!! else null
        lastTicks = ticks
        return change
    }

    fun radiansChange(): Double? {
        val change = tickChange()
        return if (change != null) toRadians(change) else null
    }

}
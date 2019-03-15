package com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.encoders

import com.david.rechargedkotlinlibrary.internal.hardware.devices.RevHub
import com.david.rechargedkotlinlibrary.internal.util.MathUtil
import com.qualcomm.robotcore.hardware.DcMotorSimple

/**
 * Created by David Lukens on 8/8/2018.
 */
open class Encoder(private val HUB: RevHub, private val PORT: Int, private val PPR: Int, direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD) {
    private var resetTicks = 0
    private var secant = 1

    init {
        setDirection(direction)
        reset()
    }

    @Throws(InterruptedException::class)
    fun getRawTicks() = HUB.getEncoder(PORT) * secant
    @Throws(InterruptedException::class)
    fun reset() {
        resetTicks = getRawTicks()
    }

    @Throws(InterruptedException::class)
    fun getTicks() = getRawTicks() - resetTicks
    @Throws(InterruptedException::class)
    fun getRadians() = toRadians(getTicks())
    @Throws(InterruptedException::class)
    fun getRawRadians() = toRadians(getRawTicks())

    @Throws(InterruptedException::class)
    fun toRadians(ticks: Int): Double {
        return (ticks.toDouble() / PPR.toDouble()) * MathUtil.TAU
    }

    @Throws(InterruptedException::class)
    fun setDirection(direction: DcMotorSimple.Direction) {
        secant = when (direction) {
            DcMotorSimple.Direction.REVERSE -> 1
            DcMotorSimple.Direction.FORWARD -> -1
        }
    }

    private var lastTicks: Int? = null

    @Throws(InterruptedException::class)
    fun tickChange(): Int? {
        var ticks = getTicks()
        var change: Int? = if (lastTicks != null) ticks - lastTicks!! else null
        lastTicks = ticks
        return change
    }

    @Throws(InterruptedException::class)
    fun radiansChange(): Double? {
        val change = tickChange()
        return if (change != null) toRadians(change) else null
    }

}
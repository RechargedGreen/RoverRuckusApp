package com.david.rechargedkotlinlibrary.internal.util

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

/**
 * Created by David Lukens on 8/8/2018.
 */
object MathUtil {
    const val TAU = 2.0 * Math.PI
    @Throws(InterruptedException::class)
    fun radiansToInches(radians: Double, radius: Double) = TAU * radians * radius
    @Throws(InterruptedException::class)
    fun norm(angle: Double, angleUnit: AngleUnit = AngleUnit.DEGREES): Double {
        var angle = if (angleUnit == AngleUnit.RADIANS) angle else Math.toRadians(angle)
        angle = (angle % TAU)
        angle = (angle + TAU) % TAU
        if (angle > Math.PI)
            angle -= TAU
        return if (angleUnit == AngleUnit.RADIANS) angle else Math.toDegrees(angle)
    }
}
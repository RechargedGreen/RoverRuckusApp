package com.david.rechargedkotlinlibrary.internal.util

/**
 * Created by David Lukens on 8/8/2018.
 */
object MathUtil {
    const val TAU = 2.0 * Math.PI
    fun radiansToInches(radians: Double, radius: Double) = TAU * radians * radius
}
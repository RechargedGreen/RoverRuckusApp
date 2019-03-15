package com.david.rechargedkotlinlibrary.internal.hardware

import com.qualcomm.robotcore.hardware.PIDCoefficients

class PIDController(val coeffs: PIDCoefficients) {
    var integral = 0.0
    var lastTime: Long? = null
    var lastError: Double? = null

    var u = 0.0
    @Throws(InterruptedException::class)
    fun update(error: Double): Double {
        val currTime = System.currentTimeMillis()
        val lastTimeCache = lastTime
        val lastErrorCache = lastError
        if (lastTimeCache != null) {
            val dt: Double = (currTime - lastTimeCache).toDouble() / 1000
            integral += error * dt
            val derivative = if (lastErrorCache != null) (error - lastErrorCache) / dt else 0.0
            u = error * coeffs.p + integral * coeffs.i + derivative * coeffs.d
            lastError = error
        }
        lastTime = currTime
        return u
    }
}
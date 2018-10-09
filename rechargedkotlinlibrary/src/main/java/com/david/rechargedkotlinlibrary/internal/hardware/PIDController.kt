package com.david.rechargedkotlinlibrary.internal.hardware

import com.qualcomm.robotcore.hardware.PIDCoefficients

class PIDController(val coeffs:PIDCoefficients) {
    var integral = 0.0
    var lastTime:Long? = null

    var u = 0.0

    fun update(error:Double):Double{
        val currTime = System.currentTimeMillis()
        val lastTimeCache = lastTime
        if(lastTimeCache != null) {
            val dt: Double = (currTime - lastTimeCache).toDouble() / 1000
            integral += error
            u = error * coeffs.p + integral * coeffs.i + error / dt * coeffs.d
        }
        lastTime = currTime
        return u
    }
}
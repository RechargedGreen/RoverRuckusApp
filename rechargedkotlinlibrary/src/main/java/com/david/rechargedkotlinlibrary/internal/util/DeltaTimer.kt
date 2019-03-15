package com.david.rechargedkotlinlibrary.internal.util

/**
 * Created by David Lukens on 1/15/2019.
 */
class DeltaTimer() {
    private var lastTime: Long = 0
    private var dt: Long = 0
    @Throws(InterruptedException::class)
    fun seconds() = milliSeconds().toDouble() / 1000.0
    @Throws(InterruptedException::class)
    fun milliSeconds(): Long {
        update()
        return dt
    }
    @Throws(InterruptedException::class)
    private fun update() {
        val currTime = System.currentTimeMillis()
        dt = currTime - lastTime
        lastTime = currTime
    }

    init {
        update()
    }
}
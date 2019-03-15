package com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors

import com.david.rechargedkotlinlibrary.internal.hardware.devices.RevHub

/**
 * Created by David Lukens on 8/9/2018.
 */
class OptimumAnalogInput(private val PORT: Int, private val HUB: RevHub) {
    @Throws(InterruptedException::class)
    fun getVoltage() = HUB.getVoltage(PORT)

    @Throws(InterruptedException::class)
    fun getMaxVoltage() = 3.3
}
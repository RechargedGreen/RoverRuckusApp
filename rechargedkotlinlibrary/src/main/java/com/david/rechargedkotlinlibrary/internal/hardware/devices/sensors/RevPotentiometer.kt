package com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors

import com.david.rechargedkotlinlibrary.internal.hardware.devices.RevHub

/**
 * Created by David Lukens on 8/9/2018.
 */
class RevPotentiometer(port: Int, hub: RevHub) {
    private val delegate = OptimumAnalogInput(port, hub)

    fun getDegrees() = delegate.getVoltage() * SCALER
    fun getRadians() = Math.toRadians(getDegrees())

    companion object {
        const val SCALER = 1 / 0.01222
    }
}
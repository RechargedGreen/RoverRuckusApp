package com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors

import com.david.rechargedkotlinlibrary.internal.hardware.devices.RevHub

/**
 * Created by David Lukens on 8/9/2018.
 */
class OptimumDigitalInput(private val HUB: RevHub, private val PORT: Int) {
    fun state() = HUB.getDigitalInput(PORT)
}
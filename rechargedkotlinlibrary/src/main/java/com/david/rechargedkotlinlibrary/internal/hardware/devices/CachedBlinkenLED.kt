package com.david.rechargedkotlinlibrary.internal.hardware.devices

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.ServoControllerEx

/**
 * Created by David Lukens on 11/16/2018.
 */
class CachedBlinkenLED(controller: ServoControllerEx, port: Int) : RevBlinkinLedDriver(controller, port) {
    var cachedPattern: RevBlinkinLedDriver.BlinkinPattern? = null

    override fun setPattern(pattern: BlinkinPattern?) {
        if (pattern != cachedPattern)
            setPattern(pattern)
        cachedPattern = pattern
    }
}
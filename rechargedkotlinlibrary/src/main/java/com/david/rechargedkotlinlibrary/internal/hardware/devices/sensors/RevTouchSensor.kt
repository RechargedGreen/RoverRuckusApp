package com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors

/**
 * Created by David Lukens on 8/9/2018.
 */
class RevTouchSensor(val delegate: OptimumDigitalInput) {
    fun pressed() = !released()
    fun released() = delegate.state()
}
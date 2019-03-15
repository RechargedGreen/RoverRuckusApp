package com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors

/**
 * Created by David Lukens on 8/9/2018.
 */
class RevTouchSensor(val delegate: OptimumDigitalInput) {
    @Throws(InterruptedException::class)
    fun pressed() = !released()
    @Throws(InterruptedException::class)
    fun released() = delegate.state()
}
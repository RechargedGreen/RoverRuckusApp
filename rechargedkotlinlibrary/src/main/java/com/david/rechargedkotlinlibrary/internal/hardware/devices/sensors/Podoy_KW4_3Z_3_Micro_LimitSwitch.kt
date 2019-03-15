package com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors

class Podoy_KW4_3Z_3_Micro_LimitSwitch(private val delegate: OptimumDigitalInput) {
    @Throws(InterruptedException::class)
    fun pressed() = delegate.state()
    @Throws(InterruptedException::class)
    fun released() = !pressed()
}
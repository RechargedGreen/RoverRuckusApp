package com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors

class Podoy_KW4_3Z_3_Micro_LimitSwitch(private val delegate: OptimumDigitalInput) {
    fun pressed() = delegate.state()
    fun released() = !pressed()
}
package com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors

class AmazonLimitSwitch(private val delegate:OptimumDigitalInput) {
    fun pressed() = delegate.state()
    fun released() = !pressed()
}
package com.david.rechargedkotlinlibrary.internal.hardware.devices

import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoController
import com.qualcomm.robotcore.util.Range

/**
 * Created by David Lukens on 8/3/2018.
 */

class CachedServo(private val delegate: Servo) : Servo {
    private var cachedPosition: Double? = null

    @Throws(InterruptedException::class)
    override fun setPosition(position: Double) {
        if (position != cachedPosition) {
            cachedPosition = position
            delegate.position = position
        }
    }

    @Throws(InterruptedException::class)
    override fun getPosition(): Double = delegate.position
    @Throws(InterruptedException::class)
    override fun scaleRange(min: Double, max: Double) = delegate.scaleRange(min, max)
    @Throws(InterruptedException::class)
    override fun setDirection(direction: Servo.Direction?) {
        delegate.direction = direction
    }

    @Throws(InterruptedException::class)
    override fun resetDeviceConfigurationForOpMode() = delegate.resetDeviceConfigurationForOpMode()
    @Throws(InterruptedException::class)
    override fun getDirection(): Servo.Direction = delegate.direction
    @Throws(InterruptedException::class)
    override fun getController(): ServoController = delegate.controller
    @Throws(InterruptedException::class)
    override fun getDeviceName(): String = delegate.deviceName
    @Throws(InterruptedException::class)
    override fun getConnectionInfo(): String = delegate.connectionInfo
    @Throws(InterruptedException::class)
    override fun getVersion(): Int = delegate.version
    @Throws(InterruptedException::class)
    override fun getPortNumber(): Int = delegate.portNumber
    @Throws(InterruptedException::class)
    override fun close() = delegate.close()
    @Throws(InterruptedException::class)
    override fun getManufacturer(): HardwareDevice.Manufacturer = delegate.manufacturer
}
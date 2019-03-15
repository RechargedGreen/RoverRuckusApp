package com.david.rechargedkotlinlibrary.internal.hardware.devices

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.ServoController

/**
 * Created by David Lukens on 10/14/2018.
 */
class CachedCRServo(private val delegate: CRServo) : CRServo {
    var powerCache: Double? = null
    @Throws(InterruptedException::class)
    override fun setPower(power: Double) {
        if (power != power) {
            powerCache = power
            delegate.power = power
        }
    }

    @Throws(InterruptedException::class)
    override fun resetDeviceConfigurationForOpMode() = delegate.resetDeviceConfigurationForOpMode()

    @Throws(InterruptedException::class)
    override fun setDirection(direction: DcMotorSimple.Direction?) {
        delegate.direction = direction
    }

    @Throws(InterruptedException::class)
    override fun getController(): ServoController = delegate.controller

    @Throws(InterruptedException::class)
    override fun getDeviceName(): String = delegate.deviceName

    @Throws(InterruptedException::class)
    override fun getConnectionInfo(): String = delegate.connectionInfo

    @Throws(InterruptedException::class)
    override fun getVersion(): Int = delegate.version

    @Throws(InterruptedException::class)
    override fun getDirection(): DcMotorSimple.Direction = delegate.direction

    @Throws(InterruptedException::class)
    override fun getPower(): Double = delegate.power

    @Throws(InterruptedException::class)
    override fun getPortNumber(): Int = delegate.portNumber

    @Throws(InterruptedException::class)
    override fun close() = delegate.close()

    @Throws(InterruptedException::class)
    override fun getManufacturer(): HardwareDevice.Manufacturer = delegate.manufacturer
}
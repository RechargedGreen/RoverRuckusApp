package com.david.rechargedkotlinlibrary.internal.hardware.devices

import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoController
import com.qualcomm.robotcore.util.Range

/**
 * Created by David Lukens on 8/3/2018.
 */

class CachedServo(private val delegate:Servo) : Servo {
    private var cachedPosition: Double? = null

    override fun setPosition(position: Double) {
        if(position != cachedPosition){
            cachedPosition = position
            delegate.position = position
        }
    }

    override fun getPosition(): Double = delegate.position
    override fun scaleRange(min: Double, max: Double) = delegate.scaleRange(min, max)
    override fun setDirection(direction: Servo.Direction?) {
        delegate.direction = direction
    }
    override fun resetDeviceConfigurationForOpMode() = delegate.resetDeviceConfigurationForOpMode()
    override fun getDirection(): Servo.Direction = delegate.direction
    override fun getController(): ServoController = delegate.controller
    override fun getDeviceName(): String = delegate.deviceName
    override fun getConnectionInfo(): String = delegate.connectionInfo
    override fun getVersion(): Int = delegate.version
    override fun getPortNumber(): Int = delegate.portNumber
    override fun close() = delegate.close()
    override fun getManufacturer(): HardwareDevice.Manufacturer = delegate.manufacturer
}
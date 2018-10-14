package com.david.rechargedkotlinlibrary.internal.hardware.devices

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.ServoController

/**
 * Created by David Lukens on 10/14/2018.
 */
class CachedCRServo (private val delegate:CRServo): CRServo{
    var powerCache:Double? = null
    override fun setPower(power: Double){
        if(power != power){
            powerCache = power
            delegate.power = power
        }
    }
    override fun resetDeviceConfigurationForOpMode() = delegate.resetDeviceConfigurationForOpMode()
    override fun setDirection(direction: DcMotorSimple.Direction?) {
        delegate.direction = direction
    }
    override fun getController(): ServoController = delegate.controller
    override fun getDeviceName(): String = delegate.deviceName
    override fun getConnectionInfo(): String = delegate.connectionInfo
    override fun getVersion(): Int = delegate.version
    override fun getDirection(): DcMotorSimple.Direction = delegate.direction
    override fun getPower(): Double = delegate.power
    override fun getPortNumber(): Int = delegate.portNumber
    override fun close() = delegate.close()
    override fun getManufacturer(): HardwareDevice.Manufacturer = delegate.manufacturer
}
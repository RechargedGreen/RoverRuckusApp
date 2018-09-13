package com.david.rechargedkotlinlibrary.internal.hardware.devices

import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoController
import com.qualcomm.robotcore.util.Range

/**
 * Created by David Lukens on 8/3/2018.
 */

class CachedServo(robot: RobotTemplate, config: String, var overrideCache: Boolean = false, min: Double = 0.0, max: Double = 1.0, direction: Servo.Direction? = Servo.Direction.FORWARD) : Servo {
    private val delegate = robot.hMap.servo.get(config)
    private var cachedPosition: Double = min - 1.0
    private var min = min
    private var max = max

    init {
        scaleRange(min, max)
        setDirection(direction)
    }

    override fun setPosition(position: Double) {
        cachedPosition = Range.clip(position, min, max)
        if (position.equals(cachedPosition).not().or(overrideCache))
            delegate.position = position
        cachedPosition = position
    }

    override fun getPosition(): Double = cachedPosition
    override fun scaleRange(min: Double, max: Double) {
        this.min = min
        this.max = max
        delegate.scaleRange(min, max)
    }

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
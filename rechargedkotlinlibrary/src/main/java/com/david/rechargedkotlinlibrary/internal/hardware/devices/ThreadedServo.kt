package com.david.rechargedkotlinlibrary.internal.hardware.devices

import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.david.rechargedkotlinlibrary.internal.hardware.management.ThreadedSubsystem
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoController

/**
 * Created by David Lukens on 8/3/2018.
 */
class ThreadedServo(robot: RobotTemplate, config: String, min: Double = 0.0, startingPosition: Double, max: Double = 1.0, private var constantlyRemindHub: Boolean = true) : ThreadedSubsystem(robot), Servo {
    private val delegate = hMap.servo.get(config)
    private var positionCache: Double = startingPosition
    private var lastPositionCache: Double = -startingPosition

    init {
        scaleRange(min, max)
    }

    override fun update() {
        var pc = positionCache
        if (pc != lastPositionCache || constantlyRemindHub)
            delegate.position = pc
        lastPositionCache = pc
    }

    override fun setPosition(position: Double) {
        positionCache = position
    }

    override fun getPosition(): Double = positionCache
    override fun scaleRange(min: Double, max: Double) = delegate.scaleRange(min, max)
    override fun setDirection(direction: Servo.Direction?) {
        delegate.direction = direction
    }

    override fun getDirection(): Servo.Direction = delegate.direction
    override fun resetDeviceConfigurationForOpMode() = delegate.resetDeviceConfigurationForOpMode()
    override fun close() = delegate.close()
    override fun getController(): ServoController = delegate.controller
    override fun getDeviceName(): String = delegate.deviceName
    override fun getConnectionInfo(): String = delegate.connectionInfo
    override fun getVersion(): Int = delegate.version
    override fun getPortNumber(): Int = delegate.portNumber
    override fun getManufacturer(): HardwareDevice.Manufacturer = delegate.manufacturer
    fun setConstantlyRemindHub(constantlyRemindHub: Boolean) {
        this.constantlyRemindHub = constantlyRemindHub
    }
}
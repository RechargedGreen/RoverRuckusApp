package com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors

import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.david.rechargedkotlinlibrary.internal.hardware.management.ThreadedSubsystem
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.DigitalChannelController
import com.qualcomm.robotcore.hardware.HardwareDevice

/**
 * Created by David Lukens on 8/3/2018.
 */
class ThreadedDigitalChannel(robot: RobotTemplate, config: String) : ThreadedSubsystem(robot), DigitalChannel {
    private var delegate = hMap.get(DigitalChannel::class.java, config)

    private var rState = false
    private var stateCache = false
    private var lastStateCache = false
    private var modeCache = DigitalChannel.Mode.INPUT
    private var lastModeCache = DigitalChannel.Mode.INPUT

    private var frozenRead: Boolean = false
    fun getFrozenRead(): Boolean = frozenRead
    fun setFrozenRead(value: Boolean) {
        frozenRead = value
    }

    init {
        frozenRead = false
        delegate.mode = modeCache
    }

    override fun start() {
    }

    override fun update() {
        val mc = modeCache
        if (mc != lastModeCache)
            delegate.mode = mc
        lastModeCache = mc

        if (mc == DigitalChannel.Mode.INPUT) {
            if (!frozenRead)
                rState = delegate.state
        } else {
            val sc = stateCache
            if (sc != lastStateCache)
                delegate.state = sc
            lastStateCache = sc
        }
    }

    override fun setState(state: Boolean) {
        stateCache = state
    }

    override fun setMode(mode: DigitalChannel.Mode?) {
        when (mode) {
            DigitalChannel.Mode.INPUT -> modeCache = DigitalChannel.Mode.INPUT
            DigitalChannel.Mode.OUTPUT -> modeCache = DigitalChannel.Mode.OUTPUT
        }
    }

    override fun setMode(mode: DigitalChannelController.Mode?) {
        when (mode) {
            DigitalChannelController.Mode.INPUT -> modeCache = DigitalChannel.Mode.INPUT
            DigitalChannelController.Mode.OUTPUT -> modeCache = DigitalChannel.Mode.OUTPUT
        }
    }

    override fun getMode(): DigitalChannel.Mode = modeCache
    override fun getState(): Boolean = rState

    override fun resetDeviceConfigurationForOpMode() = delegate.resetDeviceConfigurationForOpMode()
    override fun getDeviceName(): String = delegate.deviceName
    override fun getConnectionInfo(): String = delegate.connectionInfo
    override fun getVersion(): Int = delegate.version
    override fun close() = delegate.close()
    override fun getManufacturer(): HardwareDevice.Manufacturer = delegate.manufacturer
}
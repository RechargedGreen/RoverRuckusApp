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
    @Throws(InterruptedException::class)
    fun getFrozenRead(): Boolean = frozenRead
    @Throws(InterruptedException::class)
    fun setFrozenRead(value: Boolean) {
        frozenRead = value
    }

    init {
        frozenRead = false
        delegate.mode = modeCache
    }

    @Throws(InterruptedException::class)
    override fun start() {
    }

    @Throws(InterruptedException::class)
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

    @Throws(InterruptedException::class)
    override fun setState(state: Boolean) {
        stateCache = state
    }

    @Throws(InterruptedException::class)
    override fun setMode(mode: DigitalChannel.Mode?) {
        when (mode) {
            DigitalChannel.Mode.INPUT -> modeCache = DigitalChannel.Mode.INPUT
            DigitalChannel.Mode.OUTPUT -> modeCache = DigitalChannel.Mode.OUTPUT
        }
    }

    @Throws(InterruptedException::class)
    override fun setMode(mode: DigitalChannelController.Mode?) {
        when (mode) {
            DigitalChannelController.Mode.INPUT -> modeCache = DigitalChannel.Mode.INPUT
            DigitalChannelController.Mode.OUTPUT -> modeCache = DigitalChannel.Mode.OUTPUT
        }
    }

    @Throws(InterruptedException::class)
    override fun getMode(): DigitalChannel.Mode = modeCache
    @Throws(InterruptedException::class)
    override fun getState(): Boolean = rState

    @Throws(InterruptedException::class)
    override fun resetDeviceConfigurationForOpMode() = delegate.resetDeviceConfigurationForOpMode()
    @Throws(InterruptedException::class)
    override fun getDeviceName(): String = delegate.deviceName
    @Throws(InterruptedException::class)
    override fun getConnectionInfo(): String = delegate.connectionInfo
    @Throws(InterruptedException::class)
    override fun getVersion(): Int = delegate.version
    @Throws(InterruptedException::class)
    override fun close() = delegate.close()
    @Throws(InterruptedException::class)
    override fun getManufacturer(): HardwareDevice.Manufacturer = delegate.manufacturer
}
package com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors

import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate

/**
 * Created by David Lukens on 8/3/2018.
 */

class CachedAnalogOutput(robot: RobotTemplate, config: String) {
    private val delegate = robot.hMap.analogOutput.get(config)
    private var m: Byte = 0
    private var f: Int = 0
    private var v: Int = 0
    @Throws(InterruptedException::class)
    fun setMode(mode: Byte) {
        if (mode != m)
            delegate.setAnalogOutputMode(mode)
        m = mode
    }

    @Throws(InterruptedException::class)
    fun setFreq(freq: Int) {
        if (freq != f)
            delegate.setAnalogOutputFrequency(freq)
        f = freq
    }

    @Throws(InterruptedException::class)
    fun setVoltage(voltage: Int) {
        if (voltage != v)
            delegate.setAnalogOutputVoltage(voltage)
        v = voltage
    }
}
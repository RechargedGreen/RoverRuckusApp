package com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors

import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.david.rechargedkotlinlibrary.internal.hardware.management.ThreadedSubsystem

/**
 * Created by David Lukens on 8/3/2018.
 */

class ThreadedAnalogOutput(robot: RobotTemplate, config: String) : ThreadedSubsystem(robot) {
    private val delegate = robot.hMap.analogOutput.get(config)
    private var m: Byte = 0
    private var lm: Byte = 0
    private var f: Int = 0
    private var lf: Int = 0
    private var v: Int = 0
    private var lv: Int = 0

    @Throws(InterruptedException::class)
    override fun start() {
    }

    @Throws(InterruptedException::class)
    override fun update() {
        val mc: Byte = m
        if (mc != lm)
            delegate.setAnalogOutputMode(mc)
        lm = mc
        val fc: Int = f
        if (fc != lf)
            delegate.setAnalogOutputFrequency(fc)
        lf = fc
        val vc: Int = v
        if (vc != lv)
            delegate.setAnalogOutputVoltage(vc)
        lv = vc
    }

    @Throws(InterruptedException::class)
    fun setMode(mode: Byte) {
        m = mode
    }

    @Throws(InterruptedException::class)
    fun setFreq(freq: Int) {
        f = freq
    }

    @Throws(InterruptedException::class)
    fun setVoltage(voltage: Int) {
        v = voltage
    }
}
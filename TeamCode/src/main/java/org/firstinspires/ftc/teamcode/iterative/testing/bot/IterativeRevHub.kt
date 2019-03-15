package org.firstinspires.ftc.teamcode.iterative.testing.bot

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxNackException
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse
import com.qualcomm.robotcore.hardware.HardwareMap

class IterativeRevHub {
    lateinit var delegate: LynxModule

    @Throws(InterruptedException::class)
    fun initHardware(hMap: HardwareMap, config: String) {
        delegate = hMap.get(LynxModule::class.java, config)
        enablePhoneCharging(false)
    }

    private var response: LynxGetBulkInputDataResponse? = null

    @Throws(InterruptedException::class)
    fun pull() {
        val command = LynxGetBulkInputDataCommand(delegate)
        try {
            response = command.sendReceive()
        } catch (e: InterruptedException) {
            Thread.currentThread().interrupt()
        } catch (e: LynxNackException) {
        }
    }

    @Throws(InterruptedException::class)
    fun push() {
    }

    @Throws(InterruptedException::class)
    fun enablePhoneCharging(value: Boolean) = delegate.enablePhoneCharging(value)
    @Throws(InterruptedException::class)
    fun getEncoder(motorZ: Int): Int = response?.getEncoder(motorZ) ?: 0
    @Throws(InterruptedException::class)
    fun getDigitalInput(digitalInputZ: Int): Boolean = response?.getDigitalInput(digitalInputZ)
            ?: false
    @Throws(InterruptedException::class)
    fun getAnalogInput(inputZ: Int) = response?.getAnalogInput(inputZ) ?: 0
    @Throws(InterruptedException::class)
    fun getVelocity(motorZ: Int) = response?.getVelocity(motorZ) ?: 0.0
    @Throws(InterruptedException::class)
    fun isAtTarget(motorZ: Int): Boolean = response?.isAtTarget(motorZ) ?: false
    @Throws(InterruptedException::class)
    fun isOverCurrent(motorZ: Int): Boolean = response?.isOverCurrent(motorZ) ?: false
    @Throws(InterruptedException::class)
    fun getVoltage(inputZ: Int): Double = getAnalogInput(inputZ) / 1000.0
}
package org.firstinspires.ftc.teamcode.iterative.testing.bot

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxNackException
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse
import com.qualcomm.robotcore.hardware.HardwareMap

class IterativeRevHub {
    lateinit var delegate: LynxModule

    fun initHardware(hMap: HardwareMap, config: String) {
        delegate = hMap.get(LynxModule::class.java, config)
        enablePhoneCharging(false)
    }

    private var response: LynxGetBulkInputDataResponse? = null

    fun pull() {
        val command = LynxGetBulkInputDataCommand(delegate)
        try {
            response = command.sendReceive()
        } catch (e: InterruptedException) {
            Thread.currentThread().interrupt()
        } catch (e: LynxNackException) {
        }
    }


    fun push() {
    }

    fun enablePhoneCharging(value: Boolean) = delegate.enablePhoneCharging(value)
    fun getEncoder(motorZ: Int): Int = response?.getEncoder(motorZ) ?: 0
    fun getDigitalInput(digitalInputZ: Int): Boolean = response?.getDigitalInput(digitalInputZ)
            ?: false

    fun getAnalogInput(inputZ: Int) = response?.getAnalogInput(inputZ) ?: 0
    fun getVelocity(motorZ: Int) = response?.getVelocity(motorZ) ?: 0.0
    fun isAtTarget(motorZ: Int): Boolean = response?.isAtTarget(motorZ) ?: false
    fun isOverCurrent(motorZ: Int): Boolean = response?.isOverCurrent(motorZ) ?: false
    fun getVoltage(inputZ: Int): Double = getAnalogInput(inputZ) / 1000.0
}
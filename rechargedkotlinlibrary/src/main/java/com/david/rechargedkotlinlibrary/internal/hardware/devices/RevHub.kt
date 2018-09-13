package com.david.rechargedkotlinlibrary.internal.hardware.devices

import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxNackException
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse

class RevHub(val robot: RobotTemplate, config: String) {
    private val delegate = robot.hMap.get(LynxModule::class.java, config)
    private var response: LynxGetBulkInputDataResponse? = null

    init {
        enablePhoneCharging(false)
    }

    fun pull() {
        val command = LynxGetBulkInputDataCommand(delegate)
        try {
            response = command.sendReceive()
        } catch (e: InterruptedException) {
            Thread.currentThread().interrupt()
        } catch (e: LynxNackException) {
            // TODO: no ideal what we need to do here
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
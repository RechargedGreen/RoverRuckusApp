package com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.i2c

import com.qualcomm.hardware.lynx.LynxEmbeddedIMU
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.commands.core.LynxFirmwareVersionManager
import com.qualcomm.robotcore.hardware.I2cDeviceSynch
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple
import org.firstinspires.ftc.robotcore.internal.system.AppUtil

/**
 * Created by David Lukens on 1/23/2019.
 * inspired by https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/LynxOptimizedI2cFactory.java
 */
object LynxOptimizedI2cFactory {
    private class BetterI2cDeviceSynchImplOnSimple(simple: I2cDeviceSynchSimple, isSimpleOwned: Boolean) : I2cDeviceSynchImplOnSimple(simple, isSimpleOwned) {
        @Throws(InterruptedException::class)
        override fun setReadWindow(window: I2cDeviceSynch.ReadWindow?) {}
    }

    @Throws(InterruptedException::class)
    fun createLynxI2cDeviceSynch(module: LynxModule, bus: Int): I2cDeviceSynch = BetterI2cDeviceSynchImplOnSimple(LynxFirmwareVersionManager.createLynxI2cDeviceSynch(AppUtil.getDefContext(), module, bus), true)
    @Throws(InterruptedException::class)
    fun createLynxEmbeddedIMU(module: LynxModule, bus: Int) = LynxEmbeddedIMU(createLynxI2cDeviceSynch(module, bus))
    @Throws(InterruptedException::class)
    fun createLynxI2cColorRangeSensor(module: LynxModule, bus: Int) = LynxI2cColorRangeSensor(createLynxI2cDeviceSynch(module, bus))
}
package com.david.rechargedkotlinlibrary.internal.hardware

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.experimental.and

/**
 * Created by David Lukens on 9/30/2018.
 */
object HardwareMaker {
    object BNO055IMU{
        val AXIS_MAP_CONFIG_BYTE:Byte = 0x6 // swap x and z
        val AXIS_MAP_SIGN_BYTE:Byte = 0x1 // negate z

        fun make(hMap: HardwareMap, name:String, vertical:Boolean, mode:com.qualcomm.hardware.bosch.BNO055IMU.SensorMode):com.qualcomm.hardware.bosch.BNO055IMU {
            val imu = hMap.get(com.qualcomm.hardware.bosch.BNO055IMU::class.java, name)

            val params = com.qualcomm.hardware.bosch.BNO055IMU.Parameters()
            params.angleUnit = com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES
            params.accelUnit = com.qualcomm.hardware.bosch.BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
            params.loggingEnabled = true
            params.useExternalCrystal = true
            params.mode = mode
            params.loggingTag = "IMU"
            imu.initialize(params)

            try {
                if (vertical) {
                    imu.write8(com.qualcomm.hardware.bosch.BNO055IMU.Register.OPR_MODE, (com.qualcomm.hardware.bosch.BNO055IMU.SensorMode.CONFIG.bVal and 0x0F).toInt())// swap axes
                    Thread.sleep(100)

                    imu.write8(com.qualcomm.hardware.bosch.BNO055IMU.Register.AXIS_MAP_CONFIG, (AXIS_MAP_CONFIG_BYTE and 0x0F).toInt())// swap axes
                    imu.write8(com.qualcomm.hardware.bosch.BNO055IMU.Register.AXIS_MAP_SIGN, (AXIS_MAP_SIGN_BYTE and 0x0F).toInt())// negate axis

                    imu.write8(com.qualcomm.hardware.bosch.BNO055IMU.Register.OPR_MODE, (mode.bVal and 0x0F).toInt()) // swap back to sensor mode

                    Thread.sleep(100)
                }
            }
            catch (e:InterruptedException){
                Thread.currentThread().interrupt()
            }
            return imu
        }
    }
}
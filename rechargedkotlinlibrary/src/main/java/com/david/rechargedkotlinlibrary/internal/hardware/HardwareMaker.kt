package com.david.rechargedkotlinlibrary.internal.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.i2c.LynxOptimizedI2cFactory
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.experimental.and

/**
 * Created by David Lukens on 9/30/2018.
 */
object HardwareMaker {
    object DcMotorEx {
        @Throws(InterruptedException::class)
        fun make(hMap: HardwareMap, config: String, direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD, zeroPowerBehavior: DcMotor.ZeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE, mode: DcMotor.RunMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER): com.qualcomm.robotcore.hardware.DcMotorEx {
            val motor = hMap.get(com.qualcomm.robotcore.hardware.DcMotorEx::class.java, config)
            motor.direction = direction
            motor.zeroPowerBehavior = zeroPowerBehavior
            motor.mode = mode
            return motor
        }
    }

    object Servo {
        @Throws(InterruptedException::class)
        fun make(hMap: HardwareMap, config: String, direction: com.qualcomm.robotcore.hardware.Servo.Direction = com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD): com.qualcomm.robotcore.hardware.Servo {
            val servo = hMap.servo.get(config)
            servo.direction = direction
            return servo
        }
    }

    object CRServo {
        @Throws(InterruptedException::class)
        fun make(hMap: HardwareMap, config: String, direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD): com.qualcomm.robotcore.hardware.CRServo {
            val servo = hMap.crservo.get(config)
            servo.direction = direction
            return servo
        }
    }

    object BNO055IMU {
        val AXIS_MAP_CONFIG_BYTE: Byte = 0x6 // swap x and z
        val AXIS_MAP_SIGN_BYTE: Byte = 0x1 // negate z
        @Throws(InterruptedException::class)
        fun make(module: LynxModule, bus: Int, vertical: Boolean, mode: com.qualcomm.hardware.bosch.BNO055IMU.SensorMode): LynxEmbeddedIMU {

            val imu = LynxOptimizedI2cFactory.createLynxEmbeddedIMU(module, bus)

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
            } catch (e: InterruptedException) {
                Thread.currentThread().interrupt()
            }
            return imu
        }
    }
}
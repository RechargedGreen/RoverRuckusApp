package com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors

import com.david.rechargedkotlinlibrary.internal.hardware.devices.OptimumDcMotorEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

/**
 * Created by David Lukens on 8/7/2018.
 */
object DcMotorMaker {
    fun instantiate(configData: ConfigData, direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD, runMode: DcMotor.RunMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER, zeroPowerBehavior: DcMotor.ZeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE): OptimumDcMotorEx {
        val motor = OptimumDcMotorEx(configData)
        motor.direction = direction
        motor.mode = runMode
        motor.zeroPowerBehavior = zeroPowerBehavior
        return motor
    }
}
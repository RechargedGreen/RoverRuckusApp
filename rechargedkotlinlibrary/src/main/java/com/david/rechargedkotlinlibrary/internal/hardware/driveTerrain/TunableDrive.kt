package com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.drive.Drive
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.imu.SimplifiedBNO055

interface TunableDrive {
    @Throws(InterruptedException::class)
    fun setVel(vel: Pose2d)

    @Throws(InterruptedException::class)
    fun getMaxWheelMotorRPM(): Double

    @Throws(InterruptedException::class)
    fun getWheelRadius(): Double

    @Throws(InterruptedException::class)
    fun getWheelGearRatio(): Double

    @Throws(InterruptedException::class)
    fun getGyro(): SimplifiedBNO055

    @Throws(InterruptedException::class)
    fun getDrive(): Drive
}
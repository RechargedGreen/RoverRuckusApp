package com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.drive.Drive
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.imu.SimplifiedBNO055

interface TunableDrive {
    fun setVel(vel:Pose2d)
    fun getMaxWheelMotorRPM(): Double
    fun getWheelRadius(): Double
    fun getWheelGearRatio(): Double
    fun getGyro(): SimplifiedBNO055
    fun getDrive():Drive
}
package com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.odometry

import com.acmerobotics.roadrunner.Pose2d

/**
 * Created by David Lukens on 8/10/2018.
 */
interface Localizer {
    var biasPose: Pose2d
    fun updatePos()
    fun setPos(pos: Pose2d) {
        biasPose = -pos
    }

    fun getPos(): Pose2d = getRawPos() + biasPose
    fun resetPos() = setPos(getRawPos())
    fun getRawPos(): Pose2d
}
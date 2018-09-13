package com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.odometry

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.encoders.Encoder
import com.david.rechargedkotlinlibrary.internal.util.MathUtil

/**
 * Created by David Lukens on 8/8/2018.
 */
class Tracking2DiffWheels(var RADIUS: Double, var TICK_SCALER: Double = 1.0, var L: Encoder, var R: Encoder, val TRACK_WIDTH: Double) : Localizer {
    override var biasPose: Pose2d = Pose2d(0.0, 0.0, 0.0)

    var data = Pose2d(0.0, 0.0, 0.0)
    override fun getRawPos() = data

    override fun updatePos() {
        val dl = L.radiansChange()
        val dr = R.radiansChange()

        if (dl != null && dr != null) {
            val dc = (radiansToInches(dl) + radiansToInches(dr)) / 2
            var heading = data.heading
            data += Pose2d(Vector2d(dc * Math.cos(heading), dc * Math.sin(heading)), (radiansToInches(dr) - radiansToInches(dl)) / TRACK_WIDTH)
        }
    }

    fun radiansToInches(radians: Double) = MathUtil.radiansToInches(radians * TICK_SCALER, RADIUS)
}
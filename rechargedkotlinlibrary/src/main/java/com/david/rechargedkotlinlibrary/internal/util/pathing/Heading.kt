package com.david.rechargedkotlinlibrary.internal.util.pathing

import com.david.rechargedkotlinlibrary.internal.util.MathUtil
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

class Heading {
    private var degrees = 0.0
    private var radians = 0.0

    fun createFromRadians(radians:Double):Heading{
        val heading = Heading()
        heading.setRadians(radians)
        return heading
    }

    fun createFromDegrees(degrees:Double):Heading{
        val heading = Heading()
        heading.setDegrees(degrees)
        return heading
    }

    fun setRadians(radians: Double){
        val newRadians = MathUtil.norm(radians, AngleUnit.RADIANS)
        this.radians = newRadians
        degrees = Math.toDegrees(newRadians)
    }

    fun setDegrees(degrees: Double) {
        val newDegrees = MathUtil.norm(degrees, AngleUnit.DEGREES)
        this.degrees = newDegrees
        radians = Math.toRadians(newDegrees)
    }

    fun getRadians() = radians
    fun getDegrees() = degrees
}
package org.firstinspires.ftc.teamcode.vision

/**
 * Created by David Lukens on 10/31/2018.
 */
enum class SampleRandomizedPositions {
    LEFT,
    CENTER,
    RIGHT,
    UNKNOWN;

    fun isSide() = left() || right()

    fun right() = this == RIGHT
    fun left() = this == LEFT
    fun center() = this == CENTER
    fun unknown() = this == UNKNOWN
}
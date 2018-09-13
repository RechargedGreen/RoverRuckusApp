package com.david.rechargedkotlinlibrary.internal.opMode

import com.qualcomm.robotcore.hardware.Gamepad

/**
 * Created by David Lukens on 8/3/2018.
 */

class SimpleController(c: Gamepad) {
    val ly = -c.left_stick_y.toDouble()
    val lx = c.left_stick_x.toDouble()
    val lt = c.left_trigger.toDouble()
    val ry = -c.right_stick_y.toDouble()
    val rx = c.right_stick_x.toDouble()
    val rt = c.right_trigger.toDouble()

    val lb = c.left_bumper
    val rb = c.right_bumper

    val a = c.a
    val b = c.b
    val x = c.x
    val y = c.y
    val letter = a || b || x || y

    val lsb = c.left_stick_button
    val rsb = c.right_stick_button

    val back = c.back
    val start = c.start
    val atRest = c.atRest()
    val active = !atRest

    val dpd = c.dpad_down
    val dpl = c.dpad_left
    val dpr = c.dpad_right
    val dpu = c.dpad_up
    val dp = dpd || dpu || dpl || dpr
}
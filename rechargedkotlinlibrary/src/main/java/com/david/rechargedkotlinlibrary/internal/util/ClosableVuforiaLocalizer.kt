package com.david.rechargedkotlinlibrary.internal.util

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl

/**
 * Created by David Lukens on 8/2/2018.
 */
class ClosableVuforiaLocalizer(parameters: VuforiaLocalizer.Parameters) : VuforiaLocalizerImpl(parameters) {
    var closed: Boolean = false
    @Throws(InterruptedException::class)
    override fun close() {
        if (!closed)
            super.close()
        closed = true
    }
}
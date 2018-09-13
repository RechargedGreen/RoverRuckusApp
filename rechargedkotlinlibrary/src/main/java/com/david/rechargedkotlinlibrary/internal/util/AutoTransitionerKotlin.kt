package com.david.rechargedkotlinlibrary.internal.util

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl

/**
 * Created by David Lukens on 8/5/2018.
 */

object AutoTransitionerKotlin : Thread() {
    init {
        this.start()
    }

    private var onStop: OpMode? = null
    private var transitionTo: String? = null
    private var opModeManager: OpModeManagerImpl? = null

    override fun run() {
        try {
            while (true) {
                synchronized(this) {
                    if (onStop != null && opModeManager?.activeOpMode != onStop) {
                        Thread.sleep(1000)
                        opModeManager?.initActiveOpMode(transitionTo)
                        reset()
                    }
                }
            }
            Thread.sleep(50)
        } catch (ex: InterruptedException) {
            Log.e(FtcRobotControllerActivity.TAG, "AutoTransitioner shutdown, thread interrupted")
        }
    }

    private fun reset() {
        onStop = null
        transitionTo = null
        opModeManager = null
    }

    private fun setNewTransition(onStop: OpMode, transitionTo: String) {
        synchronized(this) {
            this.onStop = onStop
            this.transitionTo = transitionTo
            this.opModeManager = onStop.internalOpModeServices as OpModeManagerImpl
        }
    }

    fun transitionOnStop(onStop: OpMode, transitionTo: String) {
        setNewTransition(onStop, transitionTo)
    }
}
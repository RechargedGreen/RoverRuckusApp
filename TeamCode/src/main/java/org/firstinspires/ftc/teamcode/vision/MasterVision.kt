package org.firstinspires.ftc.teamcode.vision

/**
 * Created by David Lukens on 10/31/2018.
 */

class MasterVision {
    val tfLite = TFLite(this)
    fun enable(){
        tfLite.enable()
    }
    fun disable(){
        tfLite.disable()
    }
    fun stop(){
        tfLite.stop()
    }
}
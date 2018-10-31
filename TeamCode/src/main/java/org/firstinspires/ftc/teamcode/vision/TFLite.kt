package org.firstinspires.ftc.teamcode.vision

/**
 * Created by David Lukens on 10/31/2018.
 */
class TFLite(master:MasterVision) {
    fun getLastKnownSampleOrder():SampleRandomizedPositions{
        return SampleRandomizedPositions.UNKNOWN
    }

    fun enable(){}
    fun disable(){}
    fun stop(){}
}
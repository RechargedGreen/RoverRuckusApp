package org.firstinspires.ftc.teamcode.vision

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer

/**
 * Created by David Lukens on 10/31/2018.
 */

class MasterVision (parameters:VuforiaLocalizer.Parameters, val hMap: HardwareMap):Thread(){
    val vuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters)
    val tfLite = TFLite(this)
    fun enable(){
        tfLite.enable()
    }
    fun disable(){
        tfLite.disable()
    }
    fun shutdown(){
        tfLite.shutdown()
    }

    override fun run(){
        try {
            while (true){
                tfLite.updateSampleOrder()
            }
        }catch (ex:InterruptedException){
            Thread.currentThread().interrupt()
        }
    }

    init {
        start()
    }
}
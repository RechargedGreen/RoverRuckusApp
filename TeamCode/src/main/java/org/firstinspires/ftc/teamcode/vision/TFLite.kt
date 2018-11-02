package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector

/**
 * Created by David Lukens on 10/31/2018.
 */
class TFLite(master: MasterVision) {
    companion object {
        private const val TFOD_MODEL_ASSET = "RoverRuckus.tflite"
        private const val LABEL_GOLD_MINERAL = "Gold Mineral"
        private const val LABEL_SILVER_MINERAL = "Silver Mineral"
    }

    private val tfod: TFObjectDetector

    init {
        val tfodMoniterViewId = master.hMap.appContext.resources.getIdentifier("tfodMonitorViewId", "id", master.hMap.appContext.packageName)
        val parameters = TFObjectDetector.Parameters(tfodMoniterViewId)
        tfod = ClassFactory.getInstance().createTFObjectDetector(parameters, master.vuforiaLocalizer)
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL)
    }

    var lastKnownSampleOrder = SampleRandomizedPositions.UNKNOWN

    internal fun updateSampleOrder() {
        val updatedRecognitions = tfod.updatedRecognitions
        if (updatedRecognitions != null) {
            if (updatedRecognitions.size == 3) {
                var goldMineralX: Int? = null
                var silverMineral1X: Int? = null
                var silverMineral2X: Int? = null

                for (recognition in updatedRecognitions) {
                    if (recognition.label == LABEL_GOLD_MINERAL)
                        goldMineralX = recognition.left.toInt()
                    else if (silverMineral1X == null)
                        silverMineral1X = recognition.left.toInt()
                    else
                        silverMineral2X = recognition.left.toInt()
                }
                if (goldMineralX != null && silverMineral1X != null && silverMineral2X != null)
                    lastKnownSampleOrder =
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X)
                                SampleRandomizedPositions.LEFT
                            else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X)
                                SampleRandomizedPositions.RIGHT
                            else
                                SampleRandomizedPositions.CENTER
            }
        }
    }

    fun enable() {
        tfod.activate()
    }

    fun disable() {
        tfod.deactivate()
    }

    fun shutdown() {
        tfod.shutdown()
    }

}
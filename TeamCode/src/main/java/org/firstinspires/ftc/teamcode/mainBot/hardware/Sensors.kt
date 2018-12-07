package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class Sensors(val robot:HardwareClass) : MTSubsystem{
    val textToSpeech = AndroidTextToSpeech()

    private val rightRange = robot.hMap.get(DistanceSensor::class.java, "rightRange") as Rev2mDistanceSensor

    var rightRangeInches = 0.0

    override fun update() {
        rightRangeInches = rightRange.getDistance(DistanceUnit.INCH)
    }
    override fun start() {
    }

    init {
        robot.thread.addSubsystem(this)
        textToSpeech.initialize()
        textToSpeech.setLanguageAndCountry("en", "US")
    }
}
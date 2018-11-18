package org.firstinspires.ftc.teamcode.mainBot.auto

import com.acmerobotics.dashboard.config.Config
import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions
import kotlin.reflect.jvm.internal.impl.types.checker.TypeIntersector

/**
 * Created by David Lukens on 11/18/2018.
 */
@Config
abstract class RR2Auto(val angleAtStart:Double) : FluidAuto<HardwareClass>({ opMode -> HardwareClass(opMode) }){
    companion object {
        @JvmField var GOLD_HANG_ANGLE = -45.0
        @JvmField var SILVER_HANG_ANGLE = -135.0

        @JvmField var landerSampleDriveStartDistance = 300
        @JvmField var landerSampleDriveSideSampleDistance = 2000
        @JvmField var landerSampleDriveSideSampleOffSet = 40.0
        @JvmField var landerSampleDriveCenterSampleDistance = 1400

        @JvmField var parkPower = 0.15
    }

    var ORDER = SampleRandomizedPositions.UNKNOWN
    override fun tillStart() {
        ORDER = robot.vision.tfLite.lastKnownSampleOrder
        telemetry.addData("Order", ORDER)
        telemetry.update()
    }

    override fun run() {
        robot.drive.imu.setZ(angleAtStart, AngleUnit.DEGREES)
        robot.lift.deploy()
    }

    abstract fun postDeploy()

    enum class SampleCollectionType{
        LANDER_INTAKE,
        LANDER_DRIVE_PARK,
        LANDER_DRIVE_BACKUP
    }

    fun sample(sampleCollectionType: SampleCollectionType){
        when(sampleCollectionType){
            SampleCollectionType.LANDER_INTAKE -> {}
            SampleCollectionType.LANDER_DRIVE_PARK, SampleCollectionType.LANDER_DRIVE_BACKUP -> {
                val knockAngle = angleAtStart + when(ORDER){
                    SampleRandomizedPositions.UNKNOWN, SampleRandomizedPositions.CENTER -> 0.0
                    SampleRandomizedPositions.LEFT -> landerSampleDriveSideSampleOffSet
                    SampleRandomizedPositions.RIGHT -> -landerSampleDriveSideSampleOffSet
                }
                val knockDistance = when(ORDER){
                    SampleRandomizedPositions.LEFT, SampleRandomizedPositions.RIGHT -> landerSampleDriveSideSampleDistance
                    SampleRandomizedPositions.UNKNOWN, SampleRandomizedPositions.CENTER -> landerSampleDriveCenterSampleDistance
                }
                robot.drive.deadReckonPID(landerSampleDriveStartDistance, angleAtStart, DriveTerrain.AngleFollowSpeeds.SLOW)
                sleepSeconds(0.5)

                robot.drive.pidTurn(knockAngle)

                robot.drive.deadReckonPID(knockDistance, knockAngle, DriveTerrain.AngleFollowSpeeds.SLOW)

                when(sampleCollectionType) {
                    SampleCollectionType.LANDER_DRIVE_BACKUP -> robot.drive.deadReckonPID(-knockDistance, knockAngle, DriveTerrain.AngleFollowSpeeds.SLOW)
                    SampleCollectionType.LANDER_DRIVE_PARK -> {
                        if(ORDER != SampleRandomizedPositions.CENTER && ORDER != SampleRandomizedPositions.UNKNOWN)
                            robot.drive.pidTurn(0.0)
                        robot.drive.openLoopArcade(parkPower)
                        sleepSeconds(2.0)
                        robot.drive.stop()
                    }
                }
            }
        }
    }
}
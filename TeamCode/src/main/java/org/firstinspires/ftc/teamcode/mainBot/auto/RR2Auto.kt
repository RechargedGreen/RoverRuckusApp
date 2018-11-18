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
abstract class RR2Auto(val startingPosition:StartingPositions) : FluidAuto<HardwareClass>({ opMode -> HardwareClass(opMode) }){

    enum class StartingPositions(val angle:Double){
        GOLD_HANG(-45.0),
        SILVER_HANG(-135.0),
        ANY_HANG(0.0),
    }

    companion object {
        @JvmField var landerSampleDriveStartDistance = 300
        @JvmField var landerSampleDriveSideSampleDistance = 2000
        @JvmField var landerSampleDriveSideSampleOffSet = 40.0
        @JvmField var landerSampleDriveCenterSampleDistance = 1400

        @JvmField var parkPower = 0.15
    }

    var ORDER = SampleRandomizedPositions.UNKNOWN
    override fun tillStart() {
        ORDER = robot.vision.tfLite.lastKnownSampleOrder
        telemetry.addLine("must be lined up at starting position $startingPosition")
        telemetry.addData("Order", ORDER)
        telemetry.update()
    }

    override fun run() {
        robot.drive.imu.setZ(startingPosition.angle, AngleUnit.DEGREES)
        robot.lift.deploy()
    }

    abstract fun postDeploy()

    enum class SampleCollectionType{
        LANDER_INTAKE,
        LANDER_DRIVE_PARK,
        LANDER_DRIVE_BACKUP
    }

    fun sample(sampleCollectionType: SampleCollectionType){
        if(startingPosition != StartingPositions.SILVER_HANG && sampleCollectionType == SampleCollectionType.LANDER_DRIVE_PARK)
            throw IllegalArgumentException("Illegal argument $sampleCollectionType is incompatible with the $startingPosition starting position")
        when(sampleCollectionType){
            SampleCollectionType.LANDER_INTAKE -> {}
            SampleCollectionType.LANDER_DRIVE_PARK, SampleCollectionType.LANDER_DRIVE_BACKUP -> {
                val knockAngle = startingPosition.angle + when(ORDER){
                    SampleRandomizedPositions.UNKNOWN, SampleRandomizedPositions.CENTER -> 0.0
                    SampleRandomizedPositions.LEFT -> landerSampleDriveSideSampleOffSet
                    SampleRandomizedPositions.RIGHT -> -landerSampleDriveSideSampleOffSet
                }
                val knockDistance = when(ORDER){
                    SampleRandomizedPositions.LEFT, SampleRandomizedPositions.RIGHT -> landerSampleDriveSideSampleDistance
                    SampleRandomizedPositions.UNKNOWN, SampleRandomizedPositions.CENTER -> landerSampleDriveCenterSampleDistance
                }
                robot.drive.deadReckonPID(landerSampleDriveStartDistance, startingPosition.angle, DriveTerrain.AngleFollowSpeeds.SLOW)
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
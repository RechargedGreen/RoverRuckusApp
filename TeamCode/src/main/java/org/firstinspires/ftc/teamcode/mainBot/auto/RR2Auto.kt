package org.firstinspires.ftc.teamcode.mainBot.auto

import com.acmerobotics.dashboard.config.Config
import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions

/**
 * Created by David Lukens on 11/18/2018.
 */
@Config
abstract class RR2Auto(val startingPosition: StartingPositions) : FluidAuto<HardwareClass>({ opMode -> HardwareClass(opMode) }) {

    enum class StartingPositions(val angle: Double) {
        GOLD_HANG(-45.0),
        SILVER_HANG(-135.0),
        ANY_HANG(0.0),
    }

    companion object {
        @JvmField
        var landerSlowSampleDriveStartDistance = 300
        @JvmField
        var landerSlowSampleDriveSideSampleDistance = 2000
        @JvmField
        var landerSlowSampleDriveSideSampleOffSet = 40.0
        @JvmField
        var landerSlowSampleDriveCenterSampleDistance = 1400

        @JvmField
        var landerFastSampleDriveSideStartDistance = 0
        @JvmField
        var landerFastSampleDriveSideSampleDistance = 2000
        @JvmField
        var landerFastSampleDriveSideSampleOffSet = 40.0
        @JvmField
        var landerFastSampleDriveCenterSampleDistance = 0
        @JvmField
        var landerFastSampleDriveTeamMarkerCenterSampleDistance = 0

        @JvmField
        var parkPower = 0.15
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
        postDeploy()
    }

    abstract fun postDeploy()

    enum class SampleCollectionType {
        LANDER_INTAKE,
        LANDER_DRIVE_SLOW_PARK,
        LANDER_DRIVE_SLOW_BACKUP,
        LANDER_DRIVE_FAST_PARK,
        LANDER_DRIVE_FAST_BACKUP,
        LANDER_DRIVE_FAST_TEAM_MARKER,
    }

    fun sample(sampleCollectionType: SampleCollectionType) {
        if (startingPosition != StartingPositions.SILVER_HANG && (sampleCollectionType == SampleCollectionType.LANDER_DRIVE_SLOW_PARK || sampleCollectionType == SampleCollectionType.LANDER_DRIVE_FAST_PARK))
            throw IllegalArgumentException("Illegal argument $sampleCollectionType is incompatible with the $startingPosition starting position")
        when (sampleCollectionType) {
            SampleCollectionType.LANDER_INTAKE                                                                                                             -> {
            }
            SampleCollectionType.LANDER_DRIVE_FAST_PARK, SampleCollectionType.LANDER_DRIVE_FAST_BACKUP -> {
                val angle = startingPosition.angle + when(ORDER){
                    SampleRandomizedPositions.CENTER, SampleRandomizedPositions.UNKNOWN -> 0.0
                    SampleRandomizedPositions.LEFT -> landerFastSampleDriveSideSampleOffSet
                    SampleRandomizedPositions.RIGHT -> -landerFastSampleDriveSideSampleOffSet
                }
                when(ORDER){
                    SampleRandomizedPositions.CENTER, SampleRandomizedPositions.UNKNOWN  -> {
                        robot.drive.deadReckonPID(landerFastSampleDriveCenterSampleDistance, angle, DriveTerrain.AngleFollowSpeeds.FAST)
                        if(sampleCollectionType == SampleCollectionType.LANDER_DRIVE_FAST_BACKUP)
                            robot.drive.deadReckonPID(-landerFastSampleDriveCenterSampleDistance, angle, DriveTerrain.AngleFollowSpeeds.FAST)
                    }
                    SampleRandomizedPositions.LEFT, SampleRandomizedPositions.RIGHT -> {
                        robot.drive.deadReckonPID(landerFastSampleDriveSideStartDistance, startingPosition.angle, DriveTerrain.AngleFollowSpeeds.FAST, false)
                        if(ORDER == SampleRandomizedPositions.LEFT)
                            robot.drive.strafeAroundLeft(angle, stop = false)
                        else
                            robot.drive.strafeAroundRight(angle, stop = false)
                        robot.drive.deadReckonPID(landerFastSampleDriveSideSampleDistance, angle, DriveTerrain.AngleFollowSpeeds.FAST)
                        if(sampleCollectionType == SampleCollectionType.LANDER_DRIVE_FAST_BACKUP)
                            robot.drive.deadReckonPID(-landerFastSampleDriveSideSampleDistance, angle, DriveTerrain.AngleFollowSpeeds.FAST)
                    }
                }

                if(sampleCollectionType == SampleCollectionType.LANDER_DRIVE_FAST_PARK) {
                    robot.drive.pidTurn(-135.0)
                    robot.drive.openLoopArcade(parkPower)
                    sleepSeconds(2.0)
                    robot.drive.stop()
                }
            }
            SampleCollectionType.LANDER_DRIVE_FAST_TEAM_MARKER -> {

            }
            SampleCollectionType.LANDER_DRIVE_SLOW_PARK, SampleCollectionType.LANDER_DRIVE_SLOW_BACKUP                                                     -> {
                val knockAngle = startingPosition.angle + when (ORDER) {
                    SampleRandomizedPositions.UNKNOWN, SampleRandomizedPositions.CENTER -> 0.0
                    SampleRandomizedPositions.LEFT                                      -> landerSlowSampleDriveSideSampleOffSet
                    SampleRandomizedPositions.RIGHT                                     -> -landerSlowSampleDriveSideSampleOffSet
                }
                val knockDistance = when (ORDER) {
                    SampleRandomizedPositions.LEFT, SampleRandomizedPositions.RIGHT     -> landerSlowSampleDriveSideSampleDistance
                    SampleRandomizedPositions.UNKNOWN, SampleRandomizedPositions.CENTER -> landerSlowSampleDriveCenterSampleDistance
                }
                robot.drive.deadReckonPID(landerSlowSampleDriveStartDistance, startingPosition.angle, DriveTerrain.AngleFollowSpeeds.SLOW)
                sleepSeconds(0.5)

                robot.drive.pidTurn(knockAngle)

                robot.drive.deadReckonPID(knockDistance, knockAngle, DriveTerrain.AngleFollowSpeeds.SLOW)

                when (sampleCollectionType) {
                    SampleCollectionType.LANDER_DRIVE_SLOW_BACKUP -> robot.drive.deadReckonPID(-knockDistance, knockAngle, DriveTerrain.AngleFollowSpeeds.SLOW)
                    SampleCollectionType.LANDER_DRIVE_SLOW_PARK   -> {
                        if (ORDER != SampleRandomizedPositions.CENTER && ORDER != SampleRandomizedPositions.UNKNOWN)
                            robot.drive.pidTurn(-135.0)
                        robot.drive.openLoopArcade(parkPower)
                        sleepSeconds(2.0)
                        robot.drive.stop()
                    }
                }
            }
        }
    }
}
package org.firstinspires.ftc.teamcode.mainBot.auto

import com.acmerobotics.dashboard.config.Config
import com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain.DiffDrive
import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.mainBot.hardware.*
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions
import kotlin.math.absoluteValue

/**
 * Created by David Lukens on 11/18/2018.
 */
@Config
abstract class RR2Auto(val startingPosition: StartingPositions, var postDeployWait:Double = 0.0) : FluidAuto<HardwareClass>({ opMode -> HardwareClass(opMode) }) {

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
        var landerSlowSampleDriveSideSampleOffSet = 30.0
        @JvmField
        var landerSlowSampleDriveCenterSampleDistance = 1400

        @JvmField
        var landerFastSampleDriveSideStartDistance = 0
        @JvmField
        var landerFastSampleDriveSideSampleDistance = 1700
        @JvmField
        var landerFastSampleDriveSideSampleOffSet = 33.0
        @JvmField
        var landerFastSampleDriveCenterSampleDistance = 2000
        @JvmField
        var landerFastSampleDriveTeamMarkerCenterSampleDistance = 4000
        @JvmField
        var landerFastSampleDriveTeamMarkerSideSampleDistance = 1750
        @JvmField
        var teamMarkerPostSampleOffset = 45.0

        @JvmField
        var parkPower = 0.15
        @JvmField
        var intoDepotTicks = 1500
        @JvmField
        var outOfDepotTicks = 2500

        @JvmField
        var extensionSampleOffset = 45.0
        @JvmField
        var extensionSampleForwardDistance = 800

        @JvmField
        var silverSampleWallLinupDistance = 3000

        @JvmField
        var intoWallOffset = 10.0

        @JvmField
        var depotSampleCenter = 90.0
        @JvmField
        var depotSampleLeft = 60.0
        @JvmField
        var depotSampleRight = 30.0
    }

    fun intoDepotSilver(){
        robot.sensors.lineDetector.reset()
        robot.drive.startFollowingAngle_setConstants(DriveTerrain.AngleFollowSpeeds.LINE_DETECT, CompassDirection.SOUTH.degrees, true, DiffDrive.AnglePIDType.STRAIGHT)
        robot.lift.state = Lift.State.UP
        waitTill { robot.sensors.lineDetector.hasHit }
        robot.drive.stop()
    }

    var ORDER = SampleRandomizedPositions.UNKNOWN
    var lastButtonState = false
    override fun tillStart() {
        ORDER = robot.vision.tfLite.lastKnownSampleOrder
        telemetry.addLine("must be lined up at starting position $startingPosition")
        telemetry.addData("Order", ORDER)

        telemetry.addData("Wait time", "$postDeployWait seconds")
        telemetry.addLine("x to increment 1.0 seconds")
        telemetry.addLine("y to decrement 1.0 seconds")
        telemetry.addLine("lb to increment 0.1 seconds")
        telemetry.addLine("rb to decrement 0.1 seconds")

        val x = gamepad1.x
        val y = gamepad1.y
        val lb = gamepad1.left_bumper
        val rb = gamepad1.right_bumper
        val buttonState = x || y || lb || rb
        if(buttonState && !lastButtonState){
            if(x)
                postDeployWait += 1.0
            if(y)
                postDeployWait -= 1.0
            if(lb)
                postDeployWait += 0.1
            if(rb)
                postDeployWait -= 0.1
        }

        postDeployWait = Range.clip(postDeployWait, 0.0, 30.0)

        lastButtonState = buttonState

        telemetry.update()
    }

    override fun run() {
        robot.drive.imu.setZ(startingPosition.angle, AngleUnit.DEGREES)
        robot.lift.deploy()
        sleepSeconds(postDeployWait)
        robot.lift.state = Lift.State.DOWN
        postDeploy()
    }

    abstract fun postDeploy()

    fun teamMarker(stayStill:Boolean){
        robot.lift.state = Lift.State.UP
        if(!stayStill)
            robot.drive.pidTurn(CompassDirection.SOUTH_WEST.degrees)
        robot.dumper.state = Dumper.DumpState.DUMP
        waitTill { robot.lift.isFullyUp() }
        sleepSeconds(1.0)
        robot.superSystem.state = SuperSystem.State.LOWER_LIFT_AFTER_MARKER
        if(!stayStill)
            robot.drive.runTime(-0.15, 1.0)
    }

    fun prepCraterSense(){
        robot.drive.imu.resetX()
        robot.drive.imu.resetY()
    }

    fun hittingCrater() = robot.drive.imu.getX().absoluteValue + robot.drive.imu.getY().absoluteValue > 7.0

    fun silverSampleWallLinup(){
        robot.drive.pidTurn(CompassDirection.SOUTH_WEST.degrees, maxTurnPower = 0.3)
        robot.drive.deadReckonPID(-silverSampleWallLinupDistance, CompassDirection.SOUTH_WEST.degrees, DriveTerrain.AngleFollowSpeeds.FAST)
        robot.drive.pidTurn(CompassDirection.SOUTH.degrees - intoWallOffset)
        robot.drive.deadReckonPID(-1000, CompassDirection.SOUTH.degrees - intoWallOffset, DriveTerrain.AngleFollowSpeeds.SLOW)
        robot.drive.pidTurn(CompassDirection.SOUTH.degrees)
    }

    enum class SampleCollectionType {
        LANDER_DRIVE_FAST_PARK,
        LANDER_DRIVE_FAST_BACKUP,
        LANDER_DRIVE_FAST_TEAM_MARKER,
        LANDER_EXTENSION_SILVER,
        DEPOT_EXTENSION
    }

    enum class WallFollowSituation(val robotSide: RobotSide, val direction:CompassDirection, driveReverse:Boolean){
        LEFT_SAMPLE_DEPOT(RobotSide.LEFT, CompassDirection.EAST, false),
        RIGHT_SAMPLE_DEPOT(RobotSide.RIGHT, CompassDirection.NORTH, false)
    }

    enum class CompassDirection(val degrees:Double){
        NORTH(0.0),
        SOUTH(180.0),
        EAST(-90.0),
        WEST(90.0),
        NORTH_EAST(-45.0),
        NORTH_WEST(45.0),
        SOUTH_EAST(-135.0),
        SOUTH_WEST(135.0),
    }

    enum class RobotSide{
        LEFT,
        RIGHT,
        FRONT,
        BACK
    }

    fun followWall(situation:WallFollowSituation){
        when(situation){
            WallFollowSituation.LEFT_SAMPLE_DEPOT, WallFollowSituation.RIGHT_SAMPLE_DEPOT -> robot.drive.deadReckonPID(intoDepotTicks, situation.direction.degrees, DriveTerrain.AngleFollowSpeeds.SLOW)
        }
    }

    fun sample(sampleCollectionType: SampleCollectionType) {
        if (startingPosition != StartingPositions.SILVER_HANG && sampleCollectionType == SampleCollectionType.LANDER_DRIVE_FAST_PARK)
            throw IllegalArgumentException("Illegal argument $sampleCollectionType is incompatible with the $startingPosition starting position")
        robot.intake.intakeState = Intake.IntakeState.IN
        when (sampleCollectionType) {
            SampleCollectionType.DEPOT_EXTENSION -> {
                robot.drive.pidTurn(when(ORDER){
                    SampleRandomizedPositions.LEFT -> depotSampleLeft
                    SampleRandomizedPositions.RIGHT -> depotSampleRight
                    SampleRandomizedPositions.CENTER, SampleRandomizedPositions.UNKNOWN -> depotSampleCenter
                })
                robot.intake.hitSample()
            }
            SampleCollectionType.LANDER_EXTENSION_SILVER -> {
                robot.drive.deadReckonPID(extensionSampleForwardDistance, StartingPositions.SILVER_HANG.angle, DriveTerrain.AngleFollowSpeeds.SLOW)
                sleepSeconds(0.5)
                robot.drive.pidTurn(StartingPositions.SILVER_HANG.angle + when(ORDER){
                    SampleRandomizedPositions.LEFT -> extensionSampleOffset
                    SampleRandomizedPositions.CENTER, SampleRandomizedPositions.UNKNOWN -> 0.0
                    SampleRandomizedPositions.RIGHT -> -extensionSampleOffset
                }, maxTurnPower = 0.3)
                robot.intake.hitSample()
            }
            SampleCollectionType.LANDER_DRIVE_FAST_TEAM_MARKER, SampleCollectionType.LANDER_DRIVE_FAST_PARK, SampleCollectionType.LANDER_DRIVE_FAST_BACKUP -> {
                val angle = startingPosition.angle + when(ORDER){
                    SampleRandomizedPositions.CENTER, SampleRandomizedPositions.UNKNOWN -> 0.0
                    SampleRandomizedPositions.LEFT -> landerFastSampleDriveSideSampleOffSet
                    SampleRandomizedPositions.RIGHT -> -landerFastSampleDriveSideSampleOffSet
                }
                when(ORDER){
                    SampleRandomizedPositions.CENTER, SampleRandomizedPositions.UNKNOWN  -> {
                        if(sampleCollectionType == SampleCollectionType.LANDER_DRIVE_FAST_TEAM_MARKER) {
                            robot.drive.deadReckonPID(landerFastSampleDriveTeamMarkerCenterSampleDistance, angle, DriveTerrain.AngleFollowSpeeds.SLOW)
                            sleepSeconds(0.5)
                        }
                        else
                            robot.drive.deadReckonPID(landerFastSampleDriveCenterSampleDistance, angle, DriveTerrain.AngleFollowSpeeds.SLOW)
                        if(sampleCollectionType == SampleCollectionType.LANDER_DRIVE_FAST_BACKUP)
                            robot.drive.deadReckonPID(-landerFastSampleDriveCenterSampleDistance, angle, DriveTerrain.AngleFollowSpeeds.FAST)
                    }
                    SampleRandomizedPositions.LEFT, SampleRandomizedPositions.RIGHT -> {
                        robot.drive.deadReckonPID(landerFastSampleDriveSideStartDistance, startingPosition.angle, DriveTerrain.AngleFollowSpeeds.SLOW, false)
                        if(ORDER == SampleRandomizedPositions.LEFT)
                            robot.drive.strafeAroundLeft(angle, stop = false)
                        else
                            robot.drive.strafeAroundRight(angle, stop = false)
                        robot.drive.deadReckonPID(if(sampleCollectionType == SampleCollectionType.LANDER_DRIVE_FAST_TEAM_MARKER) landerFastSampleDriveTeamMarkerSideSampleDistance else landerFastSampleDriveSideSampleDistance, angle, DriveTerrain.AngleFollowSpeeds.SLOW)
                        if(sampleCollectionType == SampleCollectionType.LANDER_DRIVE_FAST_BACKUP)
                            robot.drive.deadReckonPID(-landerFastSampleDriveSideSampleDistance, angle, DriveTerrain.AngleFollowSpeeds.FAST)
                    }
                }

                if(sampleCollectionType == SampleCollectionType.LANDER_DRIVE_FAST_TEAM_MARKER){
                    when(ORDER){
                        SampleRandomizedPositions.CENTER, SampleRandomizedPositions.UNKNOWN -> {

                        }
                        SampleRandomizedPositions.LEFT, SampleRandomizedPositions.RIGHT -> {
                            var angle = startingPosition.angle + ((if(ORDER == SampleRandomizedPositions.LEFT) -1.0 else 1.0 ) * teamMarkerPostSampleOffset)
                            if(ORDER == SampleRandomizedPositions.LEFT) {
                                robot.drive.strafeAroundRight(angle)
                                followWall(WallFollowSituation.LEFT_SAMPLE_DEPOT)
                            }
                            else {
                                robot.drive.strafeAroundLeft(angle)
                                followWall(WallFollowSituation.RIGHT_SAMPLE_DEPOT)
                            }
                        }
                    }
                    robot.intake.intakeState = Intake.IntakeState.STOP
                    teamMarker(false)
                    robot.drive.pidTurn(CompassDirection.WEST.degrees - intoWallOffset)
                    robot.drive.deadReckonPID(outOfDepotTicks, CompassDirection.WEST.degrees -intoWallOffset, DriveTerrain.AngleFollowSpeeds.SLOW)
                }

                if(sampleCollectionType == SampleCollectionType.LANDER_DRIVE_FAST_PARK) {
                    robot.intake.intakeState = Intake.IntakeState.STOP
                    park(CompassDirection.SOUTH_EAST.degrees)
                }
            }
        }
        robot.intake.intakeState = Intake.IntakeState.STOP
    }

    fun park(angle:Double){
        robot.drive.pidTurn(angle)
        prepCraterSense()
        robot.drive.startFollowingAngle_setConstants(DriveTerrain.AngleFollowSpeeds.SLOW, angle, false, DiffDrive.AnglePIDType.STRAIGHT)
        waitTill { hittingCrater() }
        robot.drive.stop()
        sleepTillTime(28.0)
        robot.intake.extensionState = Intake.IntakeExtensionState.OUT
        loop()
    }
}
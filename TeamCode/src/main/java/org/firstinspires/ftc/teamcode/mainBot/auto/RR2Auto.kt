package org.firstinspires.ftc.teamcode.mainBot.auto

import com.acmerobotics.dashboard.config.Config
import com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain.DiffDrive
import com.david.rechargedkotlinlibrary.internal.opMode.FluidAuto
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.hardware.Intake
import org.firstinspires.ftc.teamcode.mainBot.hardware.Lift
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
        var landerFastSampleDriveSideSampleDistance = 2000
        @JvmField
        var landerFastSampleDriveSideSampleOffSet = 33.0
        @JvmField
        var landerFastSampleDriveCenterSampleDistance = 2000
        @JvmField
        var landerFastSampleDriveTeamMarkerCenterSampleDistance = 3500
        @JvmField
        var landerFastSampleDriveTeamMarkerSideSampleDistance = 1500
        @JvmField
        var teamMarkerPostSampleOffset = 45.0

        @JvmField
        var parkPower = 0.15
        @JvmField
        var intoDepotTicks = 1700
        @JvmField
        var outOfDepotTicks = 2500

        @JvmField
        var extensionSampleOffset = 45.0
        @JvmField
        var extensionSampleForwardDistance = 800

        @JvmField
        var silverSampleWallLinupDistance = 3000
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

    fun prepCraterSense(){
        robot.drive.imu.resetX()
        robot.drive.imu.resetY()
    }

    fun hittingCrater() = robot.drive.imu.getX().absoluteValue + robot.drive.imu.getY().absoluteValue > 7.0

    fun silverSampleWallLinup(){
        val offset = -15.0
        robot.drive.pidTurn(CompassDirection.SOUTH_WEST.degrees, maxTurnPower = 0.3)
        robot.drive.deadReckonPID(-silverSampleWallLinupDistance, CompassDirection.SOUTH_WEST.degrees, DriveTerrain.AngleFollowSpeeds.SLOW)
        robot.drive.pidTurn(CompassDirection.SOUTH.degrees + offset)
        robot.drive.deadReckonPID(-1000, CompassDirection.SOUTH.degrees + offset, DriveTerrain.AngleFollowSpeeds.SLOW)
        robot.drive.pidTurn(CompassDirection.SOUTH.degrees)
    }

    enum class SampleCollectionType {
        LANDER_DRIVE_FAST_PARK,
        LANDER_DRIVE_FAST_BACKUP,
        LANDER_DRIVE_FAST_TEAM_MARKER,
        LANDER_EXTENSION_SILVER,
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
        robot.intake.intakeState = Intake.IntakeState.OUT
        if (startingPosition != StartingPositions.SILVER_HANG && sampleCollectionType == SampleCollectionType.LANDER_DRIVE_FAST_PARK)
            throw IllegalArgumentException("Illegal argument $sampleCollectionType is incompatible with the $startingPosition starting position")
        when (sampleCollectionType) {
            SampleCollectionType.LANDER_EXTENSION_SILVER -> {
                robot.drive.deadReckonPID(extensionSampleForwardDistance, StartingPositions.SILVER_HANG.angle, DriveTerrain.AngleFollowSpeeds.SLOW)
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
                    robot.drive.pidTurn(CompassDirection.NORTH_EAST.degrees)
                    robot.drive.runTime(0.15, 1.0)
                    robot.intake.intakeState = Intake.IntakeState.SEND_MARKER
                    val offset = -4.0
                    robot.drive.pidTurn(CompassDirection.EAST.degrees + offset)
                    robot.drive.deadReckonPID(-outOfDepotTicks, CompassDirection.EAST.degrees + offset, DriveTerrain.AngleFollowSpeeds.SLOW)
                }

                if(sampleCollectionType == SampleCollectionType.LANDER_DRIVE_FAST_PARK) {
                    robot.drive.pidTurn(-135.0)
                    prepCraterSense()
                    robot.drive.startFollowingAngle_setConstants(DriveTerrain.AngleFollowSpeeds.PARK, -135.0, false, DiffDrive.AnglePIDType.STRAIGHT)
                    waitTill { hittingCrater() }
                    robot.drive.stop()
                }
            }
        }
    }
}
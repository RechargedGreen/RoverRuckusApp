package org.firstinspires.ftc.teamcode.mainBot.auto.hydra

import com.acmerobotics.dashboard.config.Config
import com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain.DiffDrive
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.mainBot.auto.RR2Auto
import org.firstinspires.ftc.teamcode.mainBot.hardware.*
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions

@Config
abstract class HydraBase : RR2Auto(StartingPositions.GOLD_HANG, 0.0, true) {

    companion object {
        @JvmField
        var extensionTicksSide = 1000
        @JvmField
        var extensionTicksCenter = 500

        @JvmField
        var leftSampleOffset = 41.0

        @JvmField
        var rightSampleOffset = 41.0

        @JvmField
        var teamMarkerOffSet = 10.0

        @JvmField
        var teamMarkerDriveTicks = 800

        @JvmField
        var fastTicks = 1800
    }

    override fun postDeploy() {
        robot.dumper.state = Dumper.DumpState.LOAD
        teamMarker()
        sample()
        toCrater()
        finish()
    }

    fun sample(){
        robot.intake.flipState = Intake.FlipState.INTAKE
        when(ORDER){
            SampleRandomizedPositions.LEFT -> {
                robot.drive.pidTurn(StartingPositions.GOLD_HANG.angle + leftSampleOffset)
                robot.intake.collectSample(extensionTicksSide, 0.5)
            }
            SampleRandomizedPositions.CENTER, SampleRandomizedPositions.UNKNOWN -> {
                robot.drive.pidTurn(StartingPositions.GOLD_HANG.angle)
                robot.intake.collectSample(extensionTicksCenter, 0.5, 1.0)
            }
            SampleRandomizedPositions.RIGHT -> {
                robot.drive.pidTurn(StartingPositions.GOLD_HANG.angle - rightSampleOffset)
                robot.intake.collectSample(extensionTicksSide, 0.5)
            }
        }

        robot.lift.state = Lift.State.UP
        if(ORDER.isSide()) robot.drive.pidTurn(StartingPositions.GOLD_HANG.angle)
        waitTill { robot.lift.isFullyUp() }

        robot.drive.runTimePID(-0.3, 0.65, StartingPositions.GOLD_HANG.angle)
        sleepSeconds(0.5)
        robot.dumper.state = Dumper.DumpState.DUMP
        sleepSeconds(0.75)

        robot.dumper.state = Dumper.DumpState.LOAD
        sleepSeconds(0.5)
        robot.lift.state = Lift.State.DOWN
    }

    fun toCrater(){
        val angle = 23.0
        robot.drive.strafeAroundLeft(angle)
        robot.drive.deadReckonPID(fastTicks, angle)

        val wallAngle = CompassDirection.WEST.degrees - 10.0
        robot.drive.pidTurn(wallAngle)
        robot.drive.deadReckonPID(1000, wallAngle, DriveTerrain.AngleFollowSpeeds.SLOW)
        robot.drive.pidTurn(CompassDirection.WEST.degrees)

        prepCraterSense()
        robot.drive.startFollowingAngle_setConstants(DriveTerrain.AngleFollowSpeeds.SLOW, CompassDirection.WEST.degrees, false, DiffDrive.AnglePIDType.STRAIGHT, 1.0)
        waitTill { hittingCrater() }
        robot.drive.deadReckonPID(-50, CompassDirection.WEST.degrees)
    }

    fun collect(){
        robot.intake.runOut(1000)
        robot.intake.flipState = Intake.FlipState.INTAKE
        robot.intake.intakeState = Intake.IntakeState.IN
        sleepSeconds(0.35)
        robot.intake.runIn(900, 0.3, 1.0)
        robot.intake.runOut(1200)
        sleepSeconds(1.0)
    }

    fun collectDefense(){
        robot.drive.pidTurn(CompassDirection.WEST.degrees + 15.0)
        robot.intake.runOut(1400)
    }

    abstract fun finish()

    fun teamMarker() {
        robot.drive.deadReckonPID(teamMarkerDriveTicks, StartingPositions.GOLD_HANG.angle, DriveTerrain.AngleFollowSpeeds.FAST)
        sleepSeconds(0.1)
        robot.drive.pidTurn(StartingPositions.GOLD_HANG.angle + teamMarkerOffSet)
        robot.superSystem.fsm = ExtendoTeamMarkerOut(robot)
        robot.superSystem.waitOnFSM()
        sleepSeconds(0.25)
        robot.intake.doMarker()
        robot.intake.flipState = Intake.FlipState.LOAD
        if(ORDER.isSide())
            sleepSeconds(0.3)
        robot.intake.retract()
    }
}

@Config
class ExtendoTeamMarkerOut(robot: HardwareClass) : SuperStructureFSM(robot) {
    companion object {
        @JvmField
        var ticksForFlipDown = 1350
        var ticksForEnd = 1450
    }

    override fun update() {
        robot.intake.flipState =
                if (robot.intake.extensionTicks() > ticksForFlipDown)
                    Intake.FlipState.INTAKE
                else
                    Intake.FlipState.LOAD
        robot.intake.extensionState = Intake.IntakeExtensionState.OUT
    }

    override fun isComplete(): Boolean {
        if(robot.intake.extensionTicks() > ticksForEnd){
            robot.intake.extensionState = Intake.IntakeExtensionState.STOP
            return true
        }
        return false
    }
}

class LoadOne(robot: HardwareClass, timeout:Double) : Load(robot, timeout) {
    override fun loaded(): Boolean = robot.superSystem.bucketSense.left() || robot.superSystem.bucketSense.right()
}

class LoadTwo(robot: HardwareClass, timeout:Double) : Load(robot, timeout) {
    override fun loaded(): Boolean = robot.superSystem.bucketSense.left() && robot.superSystem.bucketSense.right()
}

abstract class Load(robot: HardwareClass, private val timeout:Double) : SuperStructureFSM(robot) {
    private val timer = ElapsedTime()
    final override fun update() {
        robot.superSystem.bucketSense.updateCache()
        robot.intake.flipState = Intake.FlipState.LOAD
        robot.intake.extensionState = Intake.IntakeExtensionState.IN
        robot.intake.intakeState = if(robot.intake.extensionIn()) Intake.IntakeState.IN else Intake.IntakeState.STOP
    }

    abstract fun loaded():Boolean

    final override fun isComplete() : Boolean {
        if((loaded() && robot.intake.extensionIn()) || timer.seconds() > timeout){
            robot.intake.intakeState = Intake.IntakeState.STOP
            robot.intake.extensionState = Intake.IntakeExtensionState.STOP
            return true
        }
        return false
    }
}

@Autonomous
class HydraDefense : HydraBase(){
    override fun finish() = collectDefense()
}

@Autonomous
class HydraNoDefense : HydraBase(){
    override fun finish() = collect()
}

@Autonomous
class HydraNoCollect : HydraBase(){
    override fun finish() = robot.intake.runOut(600)
}
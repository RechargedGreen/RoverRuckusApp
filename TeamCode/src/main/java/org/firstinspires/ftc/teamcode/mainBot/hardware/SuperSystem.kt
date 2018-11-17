package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedBlinkenLED
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import org.firstinspires.ftc.teamcode.data.FIELD_POSITIONS
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions

class SuperSystem(val robot: HardwareClass) : MTSubsystem {
    val blinken = robot.hMap.get(RevBlinkinLedDriver::class.java, "blinken") //as CachedBlinkenLED
    val normalPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN
    val hangTimePattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED

    enum class State {
        RESET
    }

    fun setState(state: State) {
        when (state) {
            State.RESET -> {
                robot.intake.state = Intake.State.STOPPED
                robot.dumper.dumpState = Dumper.DumpState.HOLD
                robot.lift.state = Lift.State.DOWN
            }
        }
    }

    override fun update() = blinken.setPattern(if (120.0 - robot.opMode.runtime.seconds() < 10.0) hangTimePattern else normalPattern)
    override fun start() {}

    enum class SampleSituation {
        LANDER_SILVER,
        LANDER_GOLD,
        DEPOT_GOLD
    }

    fun sample(situation: SampleSituation) {
        val order = robot.vision.tfLite.lastKnownSampleOrder
        when (situation) {
            SampleSituation.LANDER_SILVER -> {
                robot.drive.waitOnTrajectory(trajectory = robot.drive.trajectoryBuilder()
                        .splineTo(FIELD_POSITIONS.SILVER_SAMPLE_FROM_LANDER_GENERAL)
                        .build()
                )
                if (order != SampleRandomizedPositions.UNKNOWN) {
                    robot.drive.waitOnTrajectory(trajectory = robot.drive.trajectoryBuilder()
                            .splineTo(when (order) {
                                          SampleRandomizedPositions.UNKNOWN -> throw Exception("Entered unexpected case while sampling $situation")
                                          SampleRandomizedPositions.LEFT    -> FIELD_POSITIONS.SILVER_SAMPLE_FROM_LANDER_LEFT
                                          SampleRandomizedPositions.CENTER  -> FIELD_POSITIONS.SILVER_SAMPLE_FROM_LANDER_CENTER
                                          SampleRandomizedPositions.RIGHT   -> FIELD_POSITIONS.SILVER_SAMPLE_FROM_LANDER_RIGHT
                                      })
                            .build())
                    robot.intake.collectSample()
                    robot.opMode.sleep(2000)// sleep for testing until sample collection is done
                }
                robot.drive.waitOnTrajectory(trajectory = robot.drive.trajectoryBuilder()
                        .turnTo(FIELD_POSITIONS.ANGLE_BEFORE_WALL_ALIGN_SPLINE_SILVER_SAMPLE_TO_DEPOT)
                        .splineTo(FIELD_POSITIONS.ALIGN_WALL_FOLLOW_SILVER_SAMPLE_TO_DEPOT)
                        .build())
            }
            SampleSituation.LANDER_GOLD   -> {

            }
            SampleSituation.DEPOT_GOLD    -> {

            }
        }
    }
}
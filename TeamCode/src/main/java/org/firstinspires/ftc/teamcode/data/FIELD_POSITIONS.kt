package org.firstinspires.ftc.teamcode.data

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d

// all the positions are based off of the red side of the field
object FIELD_POSITIONS {
    val WIDTH = 16.25 // todo get actual width
    val LENGTH = 17.0// todo get actual length
    val LANDER_SQUARE_WIDTH = 0.0 // todo get actual width
    @JvmField var GOLD_BRACKET_POS = Pose2d(positionAddedByTravel(LANDER_SQUARE_WIDTH / 2, Math.toRadians(-45.0)), Math.toRadians(-45.0))
    @JvmField var SILVER_BRACKET_POS = Pose2d(positionAddedByTravel(LANDER_SQUARE_WIDTH / 2, Math.toRadians(-135.0)), Math.toRadians(-135.0))

    @JvmField var GOLD_HANG_ANGLE = Math.toRadians(-45.0)
    @JvmField var SILVER_HANG_ANGLE = Math.toRadians(-135.0)

    @JvmField var GOLD_DEPLOY = positionAfterTravel(GOLD_BRACKET_POS.pos(), LENGTH / 2.0, GOLD_HANG_ANGLE)
    @JvmField var SILVER_DEPLOY = positionAfterTravel(SILVER_BRACKET_POS.pos(), LENGTH / 2.0, SILVER_HANG_ANGLE)

    @JvmField var DEPOT_HARD_ALIGN_FACING_OWN_CRATER = Pose2d(LENGTH / 2, WIDTH / 2, Math.toRadians(180.0))
    @JvmField var DEPOT_HARD_ALIGN_FACING_OPPONENT_CRATER = Pose2d(LENGTH / 2, WIDTH / 2, Math.toRadians(90.0))

    @JvmField var SILVER_SAMPLE_FROM_LANDER_GENERAL = Pose2d(-24.0, -24.0, Math.toRadians(0.0))
    @JvmField var SILVER_SAMPLE_FROM_LANDER_LEFT = Pose2d(SILVER_SAMPLE_FROM_LANDER_GENERAL.pos(), Math.toRadians(-90.0))
    @JvmField var SILVER_SAMPLE_FROM_LANDER_CENTER = Pose2d(SILVER_SAMPLE_FROM_LANDER_GENERAL.pos(), Math.toRadians(-135.0))
    @JvmField var SILVER_SAMPLE_FROM_LANDER_RIGHT = Pose2d(SILVER_SAMPLE_FROM_LANDER_GENERAL.pos(), Math.toRadians(180.0))

    @JvmField var GOLD_SAMPLE_FROM_DEPOT_GENERAL = Pose2d(48.0, -48.0, Math.toRadians(135.0))
    @JvmField var GOLD_SAMPLE_FROM_DEPOT_LEFT = Pose2d(GOLD_SAMPLE_FROM_DEPOT_GENERAL.pos(), Math.toRadians(90.0))
    @JvmField var GOLD_SAMPLE_FROM_DEPOT_CENTER = Pose2d(GOLD_SAMPLE_FROM_DEPOT_GENERAL.pos(), Math.toRadians(135.0))
    @JvmField var GOLD_SAMPLE_FROM_DEPOT_RIGHT = Pose2d(GOLD_SAMPLE_FROM_DEPOT_GENERAL.pos(), Math.toRadians(180.0))

    @JvmField var ALIGN_WALL_FOLLOW_SILVER_SAMPLE_TO_DEPOT = Pose2d(0.0, 0.0, Math.toRadians(0.0))// todo change for actual position

    @JvmField var ANGLE_BEFORE_WALL_ALIGN_SPLINE_SILVER_SAMPLE_TO_DEPOT = Math.toRadians(0.0)// todo change for actual position
    @JvmField var ANGLE_BEFORE_WALL_ALIGN_SPLINE_DEPOT_SAMPLE_TO_CRATER = Math.toRadians(0.0)// todo change for actual position
    @JvmField var ALIGN_WALL_FOLLOW_DEPOT_SAMPLE_TO_CRATER = Pose2d(0.0, 0.0, Math.toRadians(0.0))// todo change for actual position

    fun positionAddedByTravel(distance: Double, heading: Double):Vector2d = Vector2d(xAddedByTravel(distance, heading), yAddedByTravel(distance, heading))
    fun positionAfterTravel(start:Vector2d, distance:Double, heading:Double):Vector2d = start + positionAddedByTravel(distance, heading)
    fun xAddedByTravel(distance: Double, heading: Double) = Math.sin(heading) * distance
    fun yAddedByTravel(distance: Double, heading: Double) = Math.cos(heading) * distance
}
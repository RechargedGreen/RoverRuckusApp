package org.firstinspires.ftc.teamcode.data

import com.acmerobotics.roadrunner.Pose2d

// all the positions are based off of the red side of the field
object FIELD_POSITIONS {
    @JvmField var GOLD_DEPLOY = Pose2d(0.0, 0.0, Math.toRadians(0.0))// todo change for actual position
    @JvmField var SILVER_DEPLOY = Pose2d(0.0, 0.0, Math.toRadians(0.0))// todo change for actual position

    @JvmField var DEPOT_HARD_ALIGN = Pose2d(0.0, 0.0, Math.toRadians(0.0))// todo change for actual position

    @JvmField var SILVER_SAMPLE_FROM_LANDER_GENERAL = Pose2d(0.0, 0.0, Math.toRadians(0.0))// todo change for actual position
    @JvmField var SILVER_SAMPLE_FROM_LANDER_LEFT = Pose2d(0.0, 0.0, Math.toRadians(0.0))// todo change for actual position
    @JvmField var SILVER_SAMPLE_FROM_LANDER_CENTER = Pose2d(0.0, 0.0, Math.toRadians(0.0))// todo change for actual position
    @JvmField var SILVER_SAMPLE_FROM_LANDER_RIGHT = Pose2d(0.0, 0.0, Math.toRadians(0.0))// todo change for actual position

    @JvmField var GOLD_SAMPLE_FROM_DEPOT_GENERAL = Pose2d(0.0, 0.0, Math.toRadians(0.0))// todo change for actual position
    @JvmField var GOLD_SAMPLE_FROM_DEPOT_LEFT = Pose2d(0.0, 0.0, Math.toRadians(0.0))// todo change for actual position
    @JvmField var GOLD_SAMPLE_FROM_DEPOT_CENTER = Pose2d(0.0, 0.0, Math.toRadians(0.0))// todo change for actual position
    @JvmField var GOLD_SAMPLE_FROM_DEPOT_RIGHT = Pose2d(0.0, 0.0, Math.toRadians(0.0))// todo change for actual position

    @JvmField var ALIGN_WALL_FOLLOW_SILVER_SAMPLE_TO_DEPOT = Pose2d(0.0, 0.0, Math.toRadians(0.0))// todo change for actual position

    @JvmField var ANGLE_BEFORE_WALL_ALIGN_SPLINE_SILVER_SAMPLE_TO_DEPOT = Math.toRadians(0.0)
    @JvmField var ANGLE_BEFORE_WALL_ALIGN_SPLINE_DEPOT_SAMPLE_TO_CRATER = Math.toRadians(0.0)
    @JvmField var ALIGN_WALL_FOLLOW_DEPOT_SAMPLE_TO_CRATER = Pose2d(0.0, 0.0, Math.toRadians(0.0))
    @JvmField var GOLD_HANG_ANGLE = Math.toRadians(0.0)
    @JvmField var SILVER_HANG_ANGLE = Math.toRadians(0.0)
}
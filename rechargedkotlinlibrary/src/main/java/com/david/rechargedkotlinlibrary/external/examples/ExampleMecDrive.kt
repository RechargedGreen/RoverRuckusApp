package com.david.rechargedkotlinlibrary.external.examples

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.ConfigData
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.DcMotorMaker
import com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain.MecDrive
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

/**
 * Created by David Lukens on 8/9/2018.
 */
class ExampleMecDrive(robot: RobotTemplate) : MecDrive(
        robot = robot,
        lf = DcMotorMaker.instantiate(ConfigData(robot, 0, "lf"), direction = DcMotorSimple.Direction.REVERSE),
        lb = DcMotorMaker.instantiate(ConfigData(robot, 0, "lb"), direction = DcMotorSimple.Direction.REVERSE),
        rf = DcMotorMaker.instantiate(ConfigData(robot, 0, "rf")),
        rb = DcMotorMaker.instantiate(ConfigData(robot, 0, "rb")),
        ENCODER_SCALER = 1.5,
        mode = if (robot.opMode.isAutonomous()) DcMotor.RunMode.RUN_USING_ENCODER else DcMotor.RunMode.RUN_WITHOUT_ENCODER,
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE,
        TRACK_WIDTH = 0.0,
        WHEEL_BASE = 0.0,
        AXIAL_PID_COEFFICIENTS = PIDCoefficients(0.0, 0.0, 0.0),
        TURN_PID_COEFFICIENTS = PIDCoefficients(0.0, 0.0, 0.0),
        kV = 1.0,
        kA = 1.0,
        kStatic = 1.0,
        MAX_ACCEL = 1.0,
        MAX_TURN_ACCEL = 1.0
) {
    override fun getRawPos(): Pose2d = super.poseEstimate
    override fun updatePos() = super.updatePoseEstimate()
}
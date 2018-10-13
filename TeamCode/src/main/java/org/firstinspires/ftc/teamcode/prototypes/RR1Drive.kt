package org.firstinspires.ftc.teamcode.prototypes

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedDcMotorEx
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.ConfigData
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.imu.SimplifiedBNO055
import com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain.DiffDrive
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

/**
 * Created by David Lukens on 9/10/2018.
 */


class RR1Drive(robot: RobotTemplate) : DiffDrive(
        CROSSTRACK_PID_COEFFICIENTS = CROSSTRACK_PID_COEFFICIENTS,
        DISPLACEMENT_PID_COEFFICIENTS = DISPLACEMENT_PID_COEFFICIENTS,
        TRACK_WIDTH = TRACK_WIDTH,
        RADIUS = RADIUS,
        WHEEL_GEAR_RATIO = GEAR_RATIO,
        kV = kV,
        MAX_ACCEL = MAX_ACCEL,
        MAX_TURN_ACCEL = MAX_TURN_ACCEL,
        MAX_VEL = 1.0 / kV,
        leftMotors = arrayOf(CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "motor_drive_left_front", mode = runMode), robot.getHub(0)),
                             CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "motor_drive_left_back", mode = runMode), robot.getHub(0))),
        rightMotors = arrayOf(CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "motor_drive_right_front", mode = runMode), robot.getHub(0)),
                             CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "motor_drive_right_back", mode = runMode), robot.getHub(0))),
        robot = robot,
        imu = SimplifiedBNO055(robot.hMap.get(BNO055IMU::class.java, "imu"))
) {
    companion object Config {
        const val kV = 1.0
        const val MAX_ACCEL = 1.0
        const val MAX_TURN_ACCEL = 1.0
        const val GEAR_RATIO = 1.0
        const val RADIUS = 2.0
        const val TRACK_WIDTH = 0.0
        val CROSSTRACK_PID_COEFFICIENTS = PIDCoefficients(0.0, 0.0, 0.0)
        val DISPLACEMENT_PID_COEFFICIENTS = PIDCoefficients(0.0, 0.0, 0.0)
        val runMode = DcMotor.RunMode.RUN_USING_ENCODER
    }
}
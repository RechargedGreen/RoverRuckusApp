package org.firstinspires.ftc.teamcode.mpTuningBot

import com.acmerobotics.roadrunner.drive.TankDrive
import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple

class MPTuningDrive(private val opMode: LinearOpMode) : TankDrive(DriveConstants.TRACK_WIDTH) {
    private val lf = opMode.hardwareMap.get(DcMotorEx::class.java, "lf")
    private val lb = opMode.hardwareMap.get(DcMotorEx::class.java, "lb")
    private val rf = opMode.hardwareMap.get(DcMotorEx::class.java, "rf")
    private val rb = opMode.hardwareMap.get(DcMotorEx::class.java, "rb")

    init {
        rf.direction = DcMotorSimple.Direction.REVERSE
        rb.direction = DcMotorSimple.Direction.REVERSE

        lf.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        lb.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rf.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rb.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        lf.mode = DcMotor.RunMode.RUN_USING_ENCODER
        lb.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rf.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rb.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    val imu = HardwareMaker.BNO055IMU.make(opMode.hardwareMap.get(LynxModule::class.java, "imu"), 0, true, BNO055IMU.SensorMode.IMU)

    override fun getWheelPositions() = listOf(
            DriveConstants.encoderTicksToInches(lf.currentPosition),
            DriveConstants.encoderTicksToInches(rf.currentPosition))

    override fun setMotorPowers(left: Double, right: Double) {
        opMode.telemetry.addData("left", left)
        opMode.telemetry.addData("right", right)
        opMode.telemetry.update()
        lf.power = left
        lb.power = left
        rf.power = right
        rb.power = right
    }


    override fun getExternalHeading() = imu.angularOrientation.firstAngle.toDouble()
}
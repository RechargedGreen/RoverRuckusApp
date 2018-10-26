package org.firstinspires.ftc.teamcode.prototypes

import com.acmerobotics.roadrunner.drive.Drive
import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedDcMotorEx
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.imu.SimplifiedBNO055
import com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain.MecDrive
import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.david.rechargedkotlinlibrary.internal.opMode.PracticeTeleOp
import com.david.rechargedkotlinlibrary.internal.opMode.RechargedLinearOpMode
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(name = OrangeRR1.NAME, group = OpModeGroups.TELE_MISC)
class OrangeRR1 : PracticeTeleOp<OrangeRR1Bot>({ opMode -> OrangeRR1Bot(opMode) }) {
    override fun onLoop() = robot.drive.powerTranslation(forward = c1.ly, strafeRight = c1.lx, turnClockwise = c1.rx)

    companion object {
        const val NAME = "OrangeRR1"
    }
}

class OrangeRR1Bot(opMode: RechargedLinearOpMode<OrangeRR1Bot>) : RobotTemplate(opMode = opMode, revHubNames = arrayOf("revHub")) {
    val drive = RR1OrangeDrive(this)
    override fun autoPostInit() {}
    override fun getMaxWheelMotorRPM(): Double = 0.0
    override fun getWheelRadius(): Double = 0.0
    override fun getWheelGearRatio(): Double = 0.0
    override fun getDrive(): Drive = drive
    override fun getGyro() = SimplifiedBNO055(hMap.get(BNO055IMU::class.java, "imu"))
}

class RR1OrangeDrive(robot: RobotTemplate) : MecDrive(
        robot = robot,
        lf = CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "lf", direction = DcMotorSimple.Direction.REVERSE), robot.getHub(0)),
        lb = CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "lb", direction = DcMotorSimple.Direction.REVERSE), robot.getHub(0)),
        rf = CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "rf"), robot.getHub(0)),
        rb = CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "rb"), robot.getHub(0))
)
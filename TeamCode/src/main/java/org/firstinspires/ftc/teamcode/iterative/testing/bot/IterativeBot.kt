package org.firstinspires.ftc.teamcode.iterative.testing.bot

import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.i2c.LynxOptimizedI2cFactory
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.imu.SimplifiedBNO055
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.iterative.lib.subsystems.IterativeBotTemplate
import org.firstinspires.ftc.teamcode.iterative.lib.subsystems.SubsystemManager
import org.firstinspires.ftc.teamcode.iterative.lib.subsystems.Updatable

/**
 * Created by David Lukens on 2/15/2019.
 */
object IterativeBot : Updatable, IterativeBotTemplate {
    val drive = IterativeDrive()
    val leftHub = IterativeRevHub()
    val rightHub = IterativeRevHub()
    lateinit var imu: SimplifiedBNO055

    lateinit var extensionMotorTempBraker: DcMotor

    @Throws(InterruptedException::class)
    override fun initHardware(hMap: HardwareMap, autonomous: Boolean, subsystemManager: SubsystemManager) {
        subsystemManager.addUpdatable(this)

        leftHub.initHardware(hMap, "leftHub")
        rightHub.initHardware(hMap, "leftHub")

        imu = SimplifiedBNO055(LynxOptimizedI2cFactory.createLynxEmbeddedIMU(leftHub.delegate, 0))
        drive.initHardware(hMap, autonomous, subsystemManager)

        extensionMotorTempBraker = hMap.dcMotor.get("extension")
    }

    @Throws(InterruptedException::class)
    override fun update() {
        leftHub.pull()
        rightHub.pull()
        imu.clearCaches()
    }
}
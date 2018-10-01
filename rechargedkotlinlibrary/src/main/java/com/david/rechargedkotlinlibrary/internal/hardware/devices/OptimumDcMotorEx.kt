package com.david.rechargedkotlinlibrary.internal.hardware.devices

import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.ConfigData
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.encoders.Encoder
import com.david.rechargedkotlinlibrary.internal.hardware.management.ThreadedSubsystem
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

/**
 * Created by David Lukens on 8/7/2018.
 */
class OptimumDcMotorEx(configData: ConfigData,
                       mode: DcMotor.RunMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                       zeroPowerBehavior: DcMotor.ZeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE,
                       direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD) : DcMotorEx, ThreadedSubsystem(configData.robot) {
    val delegate = hMap.get(DcMotorEx::class.java, configData.config)
    val PORT = delegate.portNumber
    val HUB = configData.robot.getHub(configData.hub)
    private val MOTOR_TYPE = delegate.motorType
    val TICKS_PER_REV = MOTOR_TYPE.ticksPerRev
    val encoder = Encoder(configData.robot.getHub(configData.hub), delegate.portNumber, TICKS_PER_REV.toInt())

    init {
        setMode(mode)
        setZeroPowerBehavior(zeroPowerBehavior)
        setDirection(direction)
    }

    fun ticksToRadians(ticks: Int) = encoder.toRadians(ticks)
    fun getRawRadians() = encoder.getRawRadians()
    fun getRawPosition() = encoder.getRawTicks()
    fun getRadians() = encoder.getRadians()
    override fun getCurrentPosition() = encoder.getTicks()
    fun resetEncoder() = encoder.reset()

    override fun isBusy(): Boolean = HUB.isAtTarget(PORT)
    override fun getPortNumber() = PORT

    var powerCache = 0.0
    var lastPowerCache = 0.0
    override fun update() {
        val pc = powerCache
        if (pc != lastPowerCache)
            delegate.power = pc
        lastPowerCache = pc
    }

    override fun setMotorType(motorType: MotorConfigurationType?) {
        delegate.motorType = motorType
    }

    override fun setPower(power: Double) {
        powerCache = power
    }

    override fun getPower(): Double = powerCache
    override fun getMotorType(): MotorConfigurationType = MOTOR_TYPE
    override fun setMotorDisable() = delegate.setMotorDisable()
    override fun setMotorEnable() = delegate.setMotorEnable()
    override fun isMotorEnabled() = delegate.isMotorEnabled
    override fun getTargetPositionTolerance() = delegate.targetPositionTolerance
    override fun setPowerFloat() = delegate.setPowerFloat()
    override fun getPowerFloat() = delegate.powerFloat
    override fun resetDeviceConfigurationForOpMode() = delegate.resetDeviceConfigurationForOpMode()
    override fun getPIDCoefficients(mode: DcMotor.RunMode?) = delegate.getPIDCoefficients(mode)
    override fun setPIDCoefficients(mode: DcMotor.RunMode?, pidCoefficients: PIDCoefficients?) = delegate.setPIDCoefficients(mode, pidCoefficients)
    override fun getZeroPowerBehavior() = delegate.zeroPowerBehavior
    override fun getDirection() = delegate.direction
    override fun setDirection(direction: DcMotorSimple.Direction?) {
        delegate.direction = direction
        if (direction != null)
            encoder.setDirection(direction)
    }

    override fun getTargetPosition() = delegate.targetPosition
    override fun setVelocity(angularRate: Double, unit: AngleUnit?) = delegate.setVelocity(angularRate, unit)
    override fun getController() = delegate.controller
    override fun close() = delegate.close()
    override fun getVersion() = delegate.version
    override fun getDeviceName() = delegate.deviceName
    override fun setTargetPosition(position: Int) {
        delegate.targetPosition = position
    }

    override fun getMode(): DcMotor.RunMode = delegate.mode
    override fun getConnectionInfo() = delegate.connectionInfo
    override fun getVelocity(unit: AngleUnit?) = delegate.getVelocity(unit)
    override fun getManufacturer() = delegate.manufacturer
    override fun setMode(mode: DcMotor.RunMode?) {
        delegate.mode = mode
    }

    override fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior?) {
        delegate.zeroPowerBehavior = zeroPowerBehavior
    }

    override fun setTargetPositionTolerance(tolerance: Int) {
        delegate.targetPositionTolerance = tolerance
    }

    /////////////////////
    /////////////////////
    /////////////////////
    /////////////////////
    //NEW 4.0 STUFF
    /////////////////////
    /////////////////////
    /////////////////////
    /////////////////////
    override fun setVelocity(angularRate: Double) {
        delegate.velocity = angularRate
    }

    override fun getVelocity(): Double = delegate.velocity
    override fun setPIDFCoefficients(mode: DcMotor.RunMode?, pidfCoefficients: PIDFCoefficients?) = delegate.setPIDFCoefficients(mode, pidfCoefficients)

    override fun setPositionPIDFCoefficients(p: Double) = delegate.setPositionPIDFCoefficients(p)

    override fun getPIDFCoefficients(mode: DcMotor.RunMode?): PIDFCoefficients = delegate.getPIDFCoefficients(mode)

    override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) = delegate.setVelocityPIDFCoefficients(p, i, d, f)
}
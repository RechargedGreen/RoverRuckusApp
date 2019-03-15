package com.david.rechargedkotlinlibrary.internal.hardware.devices

import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.encoders.Encoder
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

/**
 * Created by David Lukens on 8/7/2018.
 */
class CachedDcMotorEx(private val delegate: DcMotorEx, private val HUB: RevHub) : DcMotorEx {
    val PORT = delegate.portNumber
    private val MOTOR_TYPE = delegate.motorType
    val TICKS_PER_REV = MOTOR_TYPE.ticksPerRev
    // todo investigate this
    val encoder = Encoder(HUB, delegate.portNumber, TICKS_PER_REV.toInt(), delegate.direction)

    @Throws(InterruptedException::class)
    override fun getCurrentPosition() = encoder.getTicks()

    @Throws(InterruptedException::class)
    override fun isBusy(): Boolean = HUB.isAtTarget(PORT)

    @Throws(InterruptedException::class)
    override fun getPortNumber() = PORT

    var powerCache = 0.0

    @Throws(InterruptedException::class)
    override fun setMotorType(motorType: MotorConfigurationType?) {
        delegate.motorType = motorType
    }

    @Throws(InterruptedException::class)
    override fun setPower(power: Double) {
        if (powerCache != power) {
            delegate.power = power
            powerCache = power
        }
    }

    @Throws(InterruptedException::class)
    override fun getPower(): Double = powerCache

    @Throws(InterruptedException::class)
    override fun getMotorType(): MotorConfigurationType = MOTOR_TYPE

    @Throws(InterruptedException::class)
    override fun setMotorDisable() = delegate.setMotorDisable()

    @Throws(InterruptedException::class)
    override fun setMotorEnable() = delegate.setMotorEnable()

    @Throws(InterruptedException::class)
    override fun isMotorEnabled() = delegate.isMotorEnabled

    @Throws(InterruptedException::class)
    override fun getTargetPositionTolerance() = delegate.targetPositionTolerance

    @Throws(InterruptedException::class)
    override fun setPowerFloat() = delegate.setPowerFloat()

    @Throws(InterruptedException::class)
    override fun getPowerFloat() = delegate.powerFloat

    @Throws(InterruptedException::class)
    override fun resetDeviceConfigurationForOpMode() = delegate.resetDeviceConfigurationForOpMode()

    @Throws(InterruptedException::class)
    override fun getPIDCoefficients(mode: DcMotor.RunMode?) = delegate.getPIDCoefficients(mode)

    @Throws(InterruptedException::class)
    override fun setPIDCoefficients(mode: DcMotor.RunMode?, pidCoefficients: PIDCoefficients?) = delegate.setPIDCoefficients(mode, pidCoefficients)

    @Throws(InterruptedException::class)
    override fun getZeroPowerBehavior() = delegate.zeroPowerBehavior

    @Throws(InterruptedException::class)
    override fun getDirection() = delegate.direction

    @Throws(InterruptedException::class)
    override fun setDirection(direction: DcMotorSimple.Direction?) {
        delegate.direction = direction
        if (direction != null)
            encoder.setDirection(direction)
    }

    @Throws(InterruptedException::class)
    override fun getTargetPosition() = delegate.targetPosition

    @Throws(InterruptedException::class)
    override fun setVelocity(angularRate: Double, unit: AngleUnit?) = delegate.setVelocity(angularRate, unit)

    @Throws(InterruptedException::class)
    override fun getController() = delegate.controller

    @Throws(InterruptedException::class)
    override fun close() = delegate.close()

    @Throws(InterruptedException::class)
    override fun getVersion() = delegate.version

    @Throws(InterruptedException::class)
    override fun getDeviceName() = delegate.deviceName

    @Throws(InterruptedException::class)
    override fun setTargetPosition(position: Int) {
        delegate.targetPosition = position
    }

    @Throws(InterruptedException::class)
    override fun getMode(): DcMotor.RunMode = delegate.mode

    @Throws(InterruptedException::class)
    override fun getConnectionInfo() = delegate.connectionInfo

    @Throws(InterruptedException::class)
    override fun getVelocity(unit: AngleUnit?) = delegate.getVelocity(unit)

    @Throws(InterruptedException::class)
    override fun getManufacturer() = delegate.manufacturer

    @Throws(InterruptedException::class)
    override fun setMode(mode: DcMotor.RunMode?) {
        delegate.mode = mode
    }

    @Throws(InterruptedException::class)
    override fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior?) {
        delegate.zeroPowerBehavior = zeroPowerBehavior
    }

    @Throws(InterruptedException::class)
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
    @Throws(InterruptedException::class)
    override fun setVelocity(angularRate: Double) {
        delegate.velocity = angularRate
    }

    @Throws(InterruptedException::class)
    override fun getVelocity(): Double = delegate.velocity

    @Throws(InterruptedException::class)
    override fun setPIDFCoefficients(mode: DcMotor.RunMode?, pidfCoefficients: PIDFCoefficients?) = delegate.setPIDFCoefficients(mode, pidfCoefficients)

    @Throws(InterruptedException::class)
    override fun setPositionPIDFCoefficients(p: Double) = delegate.setPositionPIDFCoefficients(p)

    @Throws(InterruptedException::class)
    override fun getPIDFCoefficients(mode: DcMotor.RunMode?): PIDFCoefficients = delegate.getPIDFCoefficients(mode)

    @Throws(InterruptedException::class)
    override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) = delegate.setVelocityPIDFCoefficients(p, i, d, f)
}
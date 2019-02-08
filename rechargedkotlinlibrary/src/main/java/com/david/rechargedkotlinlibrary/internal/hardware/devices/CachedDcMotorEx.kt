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

    override fun getCurrentPosition() = encoder.getTicks()

    override fun isBusy(): Boolean = HUB.isAtTarget(PORT)
    override fun getPortNumber() = PORT

    var powerCache = 0.0

    override fun setMotorType(motorType: MotorConfigurationType?) {
        delegate.motorType = motorType
    }

    override fun setPower(power: Double) {
        if (powerCache != power) {
            delegate.power = power
            powerCache = power
        }
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
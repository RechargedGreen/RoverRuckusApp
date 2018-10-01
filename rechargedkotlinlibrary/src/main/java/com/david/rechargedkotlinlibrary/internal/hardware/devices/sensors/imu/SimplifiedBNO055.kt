package com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.imu

import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.robotcore.external.navigation.*

/**
 * Created by David Lukens on 10/1/2018.
 */
class SimplifiedBNO055(private val delegate: BNO055IMU) : BNO055IMU {
    private val params = parameters
    private var useAngleCache = false

    private var zCache = 0.0
    private var xCache = 0.0
    private var yCache = 0.0

    fun clearCaches() {
        useAngleCache = false
    }

    fun checkAngleCache() {
        if (!useAngleCache) {
            val angle = getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES)
            xCache = angle.firstAngle.toDouble()
            yCache = angle.secondAngle.toDouble()
            zCache = angle.thirdAngle.toDouble()
        }
    }

    fun getZ(angleUnit: AngleUnit = AngleUnit.DEGREES): Double {
        checkAngleCache()
        return when (angleUnit) {
            AngleUnit.DEGREES -> zCache
            AngleUnit.RADIANS -> Math.toRadians(zCache)
        }
    }

    fun getX(angleUnit: AngleUnit = AngleUnit.DEGREES): Double {
        checkAngleCache()
        return when (angleUnit) {
            AngleUnit.DEGREES -> xCache
            AngleUnit.RADIANS -> Math.toRadians(xCache)
        }
    }

    fun getY(angleUnit: AngleUnit = AngleUnit.DEGREES): Double {
        checkAngleCache()
        return when (angleUnit) {
            AngleUnit.DEGREES -> yCache
            AngleUnit.RADIANS -> Math.toRadians(yCache)
        }
    }

    override fun writeCalibrationData(data: BNO055IMU.CalibrationData?) = delegate.writeCalibrationData(data)
    override fun isGyroCalibrated(): Boolean = delegate.isGyroCalibrated
    override fun write(register: BNO055IMU.Register?, data: ByteArray?) = delegate.write(register, data)
    override fun readCalibrationData(): BNO055IMU.CalibrationData = delegate.readCalibrationData()
    override fun getSystemError(): BNO055IMU.SystemError = delegate.systemError
    override fun read8(register: BNO055IMU.Register?): Byte = delegate.read8(register)
    override fun close() = delegate.close()
    override fun stopAccelerationIntegration() = delegate.stopAccelerationIntegration()
    override fun isSystemCalibrated(): Boolean = delegate.isSystemCalibrated
    override fun startAccelerationIntegration(initialPosition: Position?, initialVelocity: Velocity?, msPollInterval: Int) = delegate.startAccelerationIntegration(initialPosition, initialVelocity, msPollInterval)
    override fun getAngularVelocity(): AngularVelocity = delegate.angularVelocity
    override fun getAngularOrientation(): Orientation = delegate.angularOrientation
    override fun getAngularOrientation(reference: AxesReference?, order: AxesOrder?, angleUnit: AngleUnit?): Orientation = delegate.angularOrientation
    override fun getMagneticFieldStrength(): MagneticFlux = delegate.magneticFieldStrength
    override fun isAccelerometerCalibrated(): Boolean = delegate.isAccelerometerCalibrated
    override fun getAcceleration(): Acceleration = delegate.acceleration
    override fun getLinearAcceleration(): Acceleration = delegate.linearAcceleration
    override fun getSystemStatus(): BNO055IMU.SystemStatus = delegate.systemStatus
    override fun getParameters(): BNO055IMU.Parameters = delegate.parameters
    override fun isMagnetometerCalibrated(): Boolean = delegate.isMagnetometerCalibrated
    override fun getGravity(): Acceleration = delegate.gravity
    override fun getTemperature(): Temperature = delegate.temperature
    override fun read(register: BNO055IMU.Register?, cb: Int): ByteArray = delegate.read(register, cb)
    override fun getPosition(): Position = delegate.position
    override fun write8(register: BNO055IMU.Register?, bVal: Int) = delegate.write8(register, bVal)
    override fun getVelocity(): Velocity = delegate.velocity
    override fun getCalibrationStatus(): BNO055IMU.CalibrationStatus = delegate.calibrationStatus
    override fun getQuaternionOrientation(): Quaternion = delegate.quaternionOrientation
    override fun getOverallAcceleration(): Acceleration = delegate.overallAcceleration
    override fun initialize(parameters: BNO055IMU.Parameters): Boolean = delegate.initialize(parameters)
}
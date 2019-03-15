package com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.imu

import com.david.rechargedkotlinlibrary.internal.util.MathUtil
import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.robotcore.external.navigation.*

/**
 * Created by David Lukens on 10/1/2018.
 */
class SimplifiedBNO055(private val delegate: BNO055IMU, private val axesOrder: AxesOrder = AxesOrder.ZXY) : BNO055IMU {
    private var useAngleCache = false

    private var zCache = 0.0
    private var xCache = 0.0
    private var yCache = 0.0

    private var zBias: Double = 0.0
    private var xBias: Double = 0.0
    private var yBias: Double = 0.0
    @Throws(InterruptedException::class)
    fun setZBias(bias: Double, angleUnit: AngleUnit = AngleUnit.DEGREES) {
        zBias = if (angleUnit == AngleUnit.DEGREES) bias else Math.toRadians(bias)
    }

    @Throws(InterruptedException::class)
    fun setXBias(bias: Double, angleUnit: AngleUnit = AngleUnit.DEGREES) {
        xBias = if (angleUnit == AngleUnit.DEGREES) bias else Math.toRadians(bias)
    }

    @Throws(InterruptedException::class)
    fun setYBias(bias: Double, angleUnit: AngleUnit = AngleUnit.DEGREES) {
        yBias = if (angleUnit == AngleUnit.DEGREES) bias else Math.toRadians(bias)
    }

    @Throws(InterruptedException::class)
    fun getZBias(angleUnit: AngleUnit = AngleUnit.DEGREES): Double {
        return if (angleUnit == AngleUnit.DEGREES) zBias else Math.toRadians(zBias)
    }

    @Throws(InterruptedException::class)
    fun getXBias(angleUnit: AngleUnit = AngleUnit.DEGREES): Double {
        return if (angleUnit == AngleUnit.DEGREES) xBias else Math.toRadians(xBias)
    }

    @Throws(InterruptedException::class)
    fun getYBias(angleUnit: AngleUnit = AngleUnit.DEGREES): Double {
        return if (angleUnit == AngleUnit.DEGREES) yBias else Math.toRadians(yBias)
    }

    @Throws(InterruptedException::class)
    fun resetZ() = setZ(0.0, AngleUnit.DEGREES)

    @Throws(InterruptedException::class)
    fun resetX() = setX(0.0, AngleUnit.DEGREES)

    @Throws(InterruptedException::class)
    fun resetY() = setY(0.0, AngleUnit.DEGREES)

    @Throws(InterruptedException::class)
    fun setZ(z: Double, angleUnit: AngleUnit) = setZBias(z - getRawZ(angleUnit), angleUnit)

    @Throws(InterruptedException::class)
    fun setX(z: Double, angleUnit: AngleUnit) = setXBias(z - getRawX(angleUnit), angleUnit)

    @Throws(InterruptedException::class)
    fun setY(z: Double, angleUnit: AngleUnit) = setYBias(z - getRawY(angleUnit), angleUnit)

    @Throws(InterruptedException::class)
    fun clearCaches() {
        useAngleCache = false
    }

    @Throws(InterruptedException::class)
    fun checkAngleCache() {
        if (!useAngleCache) {
            val angle = getAngularOrientation(AxesReference.EXTRINSIC, axesOrder, AngleUnit.DEGREES)
            xCache = angle.secondAngle.toDouble()
            yCache = angle.thirdAngle.toDouble()
            zCache = angle.firstAngle.toDouble()
        }
    }

    @Throws(InterruptedException::class)
    fun getRawZ(angleUnit: AngleUnit = AngleUnit.DEGREES): Double {
        return when (angleUnit) {
            AngleUnit.DEGREES -> zCache
            AngleUnit.RADIANS -> Math.toRadians(zCache)
        }
    }

    @Throws(InterruptedException::class)
    fun getRawX(angleUnit: AngleUnit = AngleUnit.DEGREES): Double {
        return when (angleUnit) {
            AngleUnit.DEGREES -> xCache
            AngleUnit.RADIANS -> Math.toRadians(xCache)
        }
    }

    @Throws(InterruptedException::class)
    fun getRawY(angleUnit: AngleUnit = AngleUnit.DEGREES): Double {
        return when (angleUnit) {
            AngleUnit.DEGREES -> yCache
            AngleUnit.RADIANS -> Math.toRadians(yCache)
        }
    }

    @Throws(InterruptedException::class)
    fun getZ(angleUnit: AngleUnit = AngleUnit.DEGREES) = MathUtil.norm(getRawZ(angleUnit) + getZBias(angleUnit), angleUnit)

    @Throws(InterruptedException::class)
    fun getX(angleUnit: AngleUnit = AngleUnit.DEGREES) = MathUtil.norm(getRawX(angleUnit) + getXBias(angleUnit), angleUnit)

    @Throws(InterruptedException::class)
    fun getY(angleUnit: AngleUnit = AngleUnit.DEGREES) = MathUtil.norm(getRawY(angleUnit) + getYBias(angleUnit), angleUnit)

    @Throws(InterruptedException::class)
    override fun writeCalibrationData(data: BNO055IMU.CalibrationData?) = delegate.writeCalibrationData(data)

    @Throws(InterruptedException::class)
    override fun isGyroCalibrated(): Boolean = delegate.isGyroCalibrated

    @Throws(InterruptedException::class)
    override fun write(register: BNO055IMU.Register?, data: ByteArray?) = delegate.write(register, data)

    @Throws(InterruptedException::class)
    override fun readCalibrationData(): BNO055IMU.CalibrationData = delegate.readCalibrationData()

    @Throws(InterruptedException::class)
    override fun getSystemError(): BNO055IMU.SystemError = delegate.systemError

    @Throws(InterruptedException::class)
    override fun read8(register: BNO055IMU.Register?): Byte = delegate.read8(register)

    @Throws(InterruptedException::class)
    override fun close() = delegate.close()

    @Throws(InterruptedException::class)
    override fun stopAccelerationIntegration() = delegate.stopAccelerationIntegration()

    @Throws(InterruptedException::class)
    override fun isSystemCalibrated(): Boolean = delegate.isSystemCalibrated

    @Throws(InterruptedException::class)
    override fun startAccelerationIntegration(initialPosition: Position?, initialVelocity: Velocity?, msPollInterval: Int) = delegate.startAccelerationIntegration(initialPosition, initialVelocity, msPollInterval)

    @Throws(InterruptedException::class)
    override fun getAngularVelocity(): AngularVelocity = delegate.angularVelocity

    @Throws(InterruptedException::class)
    override fun getAngularOrientation(): Orientation = delegate.angularOrientation

    @Throws(InterruptedException::class)
    override fun getAngularOrientation(reference: AxesReference?, order: AxesOrder?, angleUnit: AngleUnit?): Orientation = delegate.angularOrientation

    @Throws(InterruptedException::class)
    override fun getMagneticFieldStrength(): MagneticFlux = delegate.magneticFieldStrength

    @Throws(InterruptedException::class)
    override fun isAccelerometerCalibrated(): Boolean = delegate.isAccelerometerCalibrated

    @Throws(InterruptedException::class)
    override fun getAcceleration(): Acceleration = delegate.acceleration

    @Throws(InterruptedException::class)
    override fun getLinearAcceleration(): Acceleration = delegate.linearAcceleration

    @Throws(InterruptedException::class)
    override fun getSystemStatus(): BNO055IMU.SystemStatus = delegate.systemStatus

    @Throws(InterruptedException::class)
    override fun getParameters(): BNO055IMU.Parameters = delegate.parameters

    @Throws(InterruptedException::class)
    override fun isMagnetometerCalibrated(): Boolean = delegate.isMagnetometerCalibrated

    @Throws(InterruptedException::class)
    override fun getGravity(): Acceleration = delegate.gravity

    @Throws(InterruptedException::class)
    override fun getTemperature(): Temperature = delegate.temperature

    @Throws(InterruptedException::class)
    override fun read(register: BNO055IMU.Register?, cb: Int): ByteArray = delegate.read(register, cb)

    @Throws(InterruptedException::class)
    override fun getPosition(): Position = delegate.position

    @Throws(InterruptedException::class)
    override fun write8(register: BNO055IMU.Register?, bVal: Int) = delegate.write8(register, bVal)

    @Throws(InterruptedException::class)
    override fun getVelocity(): Velocity = delegate.velocity

    @Throws(InterruptedException::class)
    override fun getCalibrationStatus(): BNO055IMU.CalibrationStatus = delegate.calibrationStatus

    @Throws(InterruptedException::class)
    override fun getQuaternionOrientation(): Quaternion = delegate.quaternionOrientation

    @Throws(InterruptedException::class)
    override fun getOverallAcceleration(): Acceleration = delegate.overallAcceleration

    @Throws(InterruptedException::class)
    override fun initialize(parameters: BNO055IMU.Parameters): Boolean = delegate.initialize(parameters)
}
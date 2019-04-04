package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedDcMotorEx
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.OptimumDigitalInput
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.RevTouchSensor
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range

class Intake(val robot: HardwareClass) : MTSubsystem {

    var brakingExtension = true

    companion object {
        var spoolSize = 1.25
        var gearRatio = 28.0 / 44.0
        var cpr = 560.0
    }

    enum class IntakeState(internal val power: Double) {
        IN(1.0),
        OUT(-1.0),
        MARKER(OUT.power * 0.5),
        STOP(0.0)
    }

    var intakePower = 0.0

    private val intakeFlipR = HardwareMaker.Servo.make(robot.hMap, "intakeFlipR")
    private val intakeFlipL = HardwareMaker.Servo.make(robot.hMap, "intakeFlipL")
    var flipState = FlipState.INTAKE

    @Throws(InterruptedException::class)
    fun doMarker() {
        intakeState = IntakeState.MARKER
        robot.opMode.sleepSeconds(0.5)
        intakeState = IntakeState.STOP
    }

    enum class FlipState(internal val pos: Double) {
        LOAD(1.0),
        INTAKE(0.0)
    }

    enum class ExtensionState {
        IN,
        OUT
    }

    enum class ExtensionControlState {
        AUTO,
        MANUAL_DANGER,
        MANUAL_SAFE
    }

    enum class IntakeExtensionState(internal val power: Double) {
        IN(-1.0),
        OUT(1.0),
        STOP(0.0)
    }

    var intakeState = IntakeState.STOP
        set(value) {
            intakePower = value.power
            field = value
        }
    var extensionControlState = Intake.ExtensionControlState.AUTO
    var extensionState = IntakeExtensionState.STOP
        set(value) {
            field = value
            extensionControlState = ExtensionControlState.AUTO
        }

    private val intakeMotor = CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "intake", DcMotorSimple.Direction.REVERSE), robot.getHub(0))
    private val extensionMotor = CachedDcMotorEx(HardwareMaker.DcMotorEx.make(robot.hMap, "extension", zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE, direction = DcMotorSimple.Direction.REVERSE), robot.getHub(1))

    private var manualExtensionPower = 0.0

    @Throws(InterruptedException::class)
    fun manualPowerExtension(power: Double, useFailSafe: Boolean) {
        manualExtensionPower = power
        extensionControlState = if (useFailSafe) ExtensionControlState.MANUAL_SAFE else ExtensionControlState.MANUAL_DANGER
    }

    @Throws(InterruptedException::class)
    fun extensionTicks() = extensionMotor.encoder.getRawTicks()

    @Throws(InterruptedException::class)
    fun extensionInches() = (extensionReset - extensionTicks()) / ticksPerInch()

    @Throws(InterruptedException::class)
    fun ticksPerInch() = gearRatio * cpr / spoolSize

    @Throws(InterruptedException::class)
    fun hitSample() {
        val timer = ElapsedTime()
        extensionState = IntakeExtensionState.OUT
        robot.opMode.waitTill { robot.intake.extensionOut() || timer.seconds() > 2.0 }
        extensionState = IntakeExtensionState.IN
        robot.opMode.waitTill { robot.intake.extensionIn() || timer.seconds() > 3.0 }
    }

    private var extensionReset = 0

    @Throws(InterruptedException::class)
    override fun update() {
        extensionMotor.zeroPowerBehavior = if (brakingExtension) DcMotor.ZeroPowerBehavior.BRAKE else DcMotor.ZeroPowerBehavior.FLOAT
        if (extensionIn())
            extensionReset = extensionTicks()
        internalPowerIntake(intakePower)
        internalPowerExtension(when (extensionControlState) {
            ExtensionControlState.AUTO -> extensionState.power
            ExtensionControlState.MANUAL_DANGER, ExtensionControlState.MANUAL_SAFE -> manualExtensionPower
        }, extensionControlState != ExtensionControlState.MANUAL_DANGER)

        internalSetIntakeFlipPos(flipState.pos)
    }

    @Throws(InterruptedException::class)
    fun internalSetIntakeFlipPos(position: Double) {
        intakeFlipR.position = position
        intakeFlipL.position = 1.0 - position
    }

    @Throws(InterruptedException::class)
    override fun start() {
    }

    @Throws(InterruptedException::class)
    private fun internalPowerIntake(power: Double) {
        intakeMotor.power = power
    }

    @Throws(InterruptedException::class)
    private fun internalPowerExtension(power: Double, useFailSafe: Boolean) {
        extensionMotor.power = Range.clip(power, if (useFailSafe && extensionIn()) 0.0 else -1.0, if (useFailSafe && extensionOut()) 0.0 else 1.0)
    }

    init {
        robot.thread.addSubsystem(this)
    }

    private val inTouch = RevTouchSensor(OptimumDigitalInput(robot.getHub(1), 5))
    @Throws(InterruptedException::class)
    fun extensionIn() = inTouch.pressed()

    @Throws(InterruptedException::class)
    fun extensionOut() = false
}
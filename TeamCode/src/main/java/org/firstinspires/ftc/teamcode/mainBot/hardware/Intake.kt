package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedServo
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.OptimumDigitalInput
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.RevTouchSensor
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range

class Intake(val robot: HardwareClass) : MTSubsystem {
    enum class IntakeState(internal val power: Double) {
        IN(1.0),
        OUT(-1.0),
        STOP(0.0)
    }

    private val intakeFlip = CachedServo(HardwareMaker.Servo.make(robot.hMap, "intakeFlip"))
    var flipState = FlipState.INTAKE
    enum class FlipState(internal val pos:Double){
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
    var extensionControlState = Intake.ExtensionControlState.AUTO
    var extensionState = IntakeExtensionState.STOP
        set(value) {
            field = value
            extensionControlState = ExtensionControlState.AUTO
        }

    private val intakeMotor = HardwareMaker.DcMotorEx.make(robot.hMap, "intake", DcMotorSimple.Direction.REVERSE)
    private val extensionMotor = HardwareMaker.DcMotorEx.make(robot.hMap, "extension")

    private var manualExtensionPower = 0.0

    fun manualPowerExtension(power: Double, useFailSafe: Boolean) {
        manualExtensionPower = power
        extensionControlState = if (useFailSafe) ExtensionControlState.MANUAL_SAFE else ExtensionControlState.MANUAL_DANGER
    }

    fun hitSample(){
        val timer = ElapsedTime()
        extensionState = IntakeExtensionState.OUT
        robot.opMode.waitTill { robot.intake.extensionOut() || timer.seconds() > 2.0 }
        extensionState = IntakeExtensionState.IN
        robot.opMode.waitTill { robot.intake.extensionIn() || timer.seconds() > 3.0 }
    }

    override fun update() {
        internalPowerIntake(intakeState.power)
        internalPowerExtension(when (extensionControlState) {
            ExtensionControlState.AUTO -> extensionState.power
            ExtensionControlState.MANUAL_DANGER, ExtensionControlState.MANUAL_SAFE -> manualExtensionPower
        }, extensionControlState != ExtensionControlState.MANUAL_DANGER)

        internalSetIntakeFlipPos(flipState.pos)
    }

    fun internalSetIntakeFlipPos(position:Double){
        intakeFlip.position = position
    }

    override fun start() {
    }

    private fun internalPowerIntake(power: Double) {
        intakeMotor.power = power
    }

    private fun internalPowerExtension(power: Double, useFailSafe: Boolean) {
        extensionMotor.power = Range.clip(power, if (useFailSafe && extensionIn()) 0.0 else -1.0, if (useFailSafe && extensionOut()) 0.0 else 1.0)
    }

    init {
        robot.thread.addSubsystem(this)
        extensionState = IntakeExtensionState.IN
    }

    private val inTouch = RevTouchSensor(OptimumDigitalInput(robot.getHub(1), 5))

    fun extensionIn() = inTouch.pressed()
    fun extensionOut() = false
}
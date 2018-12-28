package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.david.rechargedkotlinlibrary.internal.hardware.devices.CachedServo
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.OptimumDigitalInput
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.Podoy_KW4_3Z_3_Micro_LimitSwitch
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.RevTouchSensor
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range

class Intake(val robot: HardwareClass) : MTSubsystem {
    enum class IntakeState(internal val power: Double) {
        IN(1.0),
        OUT(-1.0),
        STOP(0.0),
        SEND_MARKER(OUT.power)
    }

    private val intakeFlip = CachedServo(HardwareMaker.Servo.make(robot.hMap, "intakeFlip"))
    private val intakeFlap = CachedServo(HardwareMaker.Servo.make(robot.hMap, "intakeFlap"))
    private val intakeFlipUp = 1.0
    private val intakeFlipDown = 0.0
    private val intakeGateRelease = 0.0
    private val intakeGateClose = 1.0
    var intakeBucketState = IntakeBucketState.UP
    enum class IntakeBucketState{
        INTAKE,
        UP,
        LOAD_BUCKET
    }

    enum class SortState {
        BLIND,
        BLOCKS
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
        OUT(1.0)
    }

    val sendingTimer = ElapsedTime()

    var intakeState = IntakeState.STOP
        set(value){
            if(value == IntakeState.SEND_MARKER)
                sendingTimer.reset()
            field = value
        }
    var extensionControlState = Intake.ExtensionControlState.AUTO
    var sortState: SortState = SortState.BLIND
    var extensionState = IntakeExtensionState.IN
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
        robot.intake.manualPowerExtension(1.0, true)
        robot.opMode.sleepSeconds(2.0)
        val timer = ElapsedTime()
        robot.intake.extensionState = IntakeExtensionState.IN
        robot.opMode.waitTill { robot.intake.extensionIn() || timer.seconds() > 3.0 }

        /*extensionState = IntakeExtensionState.OUT
        robot.opMode.waitTill { extensionOut() }
        extensionState = IntakeExtensionState.IN
        robot.opMode.waitTill { extensionIn() }*/
    }

    override fun update() {
        if(intakeState == IntakeState.SEND_MARKER && sendingTimer.seconds() > 2.0)
            intakeState = IntakeState.STOP
        internalPowerIntake(intakeState.power)
        internalPowerExtension(when (extensionControlState) {
            ExtensionControlState.AUTO -> extensionState.power
            ExtensionControlState.MANUAL_DANGER, ExtensionControlState.MANUAL_SAFE -> manualExtensionPower
        }, extensionControlState != ExtensionControlState.MANUAL_DANGER)
        internalSetSort(sortState)

        when(intakeBucketState){
            IntakeBucketState.INTAKE -> {
                internalSetIntakeFlipPos(intakeFlipDown)
                internalSetFlapPos(intakeGateClose)
            }
            IntakeBucketState.UP -> {
                internalSetIntakeFlipPos(intakeFlipUp)
                internalSetFlapPos(intakeGateClose)
            }
            IntakeBucketState.LOAD_BUCKET -> {
                internalSetIntakeFlipPos(intakeFlipUp)
                internalSetFlapPos(intakeGateRelease)
            }
        }
    }

    fun internalSetFlapPos(position: Double){
        intakeFlap.position = position
    }
    fun internalSetIntakeFlipPos(position:Double){
        intakeFlip.position = position
    }

    override fun start() {
    }

    private fun internalPowerIntake(power: Double) {
        intakeMotor.power = power
    }

    private fun internalSetSort(state:SortState){
    }

    private fun internalPowerExtension(power: Double, useFailSafe: Boolean) {
        extensionMotor.power = Range.clip(power, if (useFailSafe && extensionIn()) 0.0 else -1.0, if (useFailSafe && extensionOut()) 0.0 else 1.0)
    }

    init {
        robot.thread.addSubsystem(this)
        extensionState = IntakeExtensionState.IN
    }

    private val inTouch = RevTouchSensor(OptimumDigitalInput(robot.getHub(1), 5))

    private fun extensionIn() = inTouch.pressed()
    private fun extensionOut() = false
}
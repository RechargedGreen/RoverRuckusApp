package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.HardwareMaker
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.OptimumDigitalInput
import com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.Podoy_KW4_3Z_3_Micro_LimitSwitch
import com.david.rechargedkotlinlibrary.internal.hardware.management.MTSubsystem
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range

class Intake(val robot: HardwareClass) : MTSubsystem {
    enum class IntakeState(internal val power: Double) {
        IN(1.0),
        OUT(-1.0),
        STOP(0.0),
        SEND_MARKER(OUT.power)
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

    private val intakeMotor = HardwareMaker.DcMotorEx.make(robot.hMap, "intake")
    private val extensionMotor = HardwareMaker.DcMotorEx.make(robot.hMap, "extension")

    private var manualExtensionPower = 0.0

    fun manualPowerExtension(power: Double, useFailSafe: Boolean) {
        manualExtensionPower = power
        extensionControlState = if (useFailSafe) ExtensionControlState.MANUAL_SAFE else ExtensionControlState.MANUAL_DANGER
    }

    fun hitSample(){
        robot.opMode.sleepSeconds(2.0)//todo make actual
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
    }

    private val leftInLimit = Podoy_KW4_3Z_3_Micro_LimitSwitch(OptimumDigitalInput(robot.getHub(1), 0))// todo get port numbers
    private val leftOutLimit = Podoy_KW4_3Z_3_Micro_LimitSwitch(OptimumDigitalInput(robot.getHub(1), 0))
    private val rightInLimit = Podoy_KW4_3Z_3_Micro_LimitSwitch(OptimumDigitalInput(robot.getHub(1), 0))
    private val rightOutLimit = Podoy_KW4_3Z_3_Micro_LimitSwitch(OptimumDigitalInput(robot.getHub(1), 0))

    private fun extensionIn() = leftInLimit.pressed() && rightInLimit.pressed()
    private fun extensionOut() = leftOutLimit.pressed() && rightOutLimit.pressed()
}
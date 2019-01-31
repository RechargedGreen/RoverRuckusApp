package org.firstinspires.ftc.teamcode.mainBot.teleOp

import com.david.rechargedkotlinlibrary.internal.opMode.PracticeTeleOp
import com.david.rechargedkotlinlibrary.internal.util.BooleanToggle
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.hardware.Dumper
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.hardware.Intake
import org.firstinspires.ftc.teamcode.mainBot.hardware.Lift
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups
import kotlin.math.absoluteValue

//@TeleOp(group = OpModeGroups.TELEOP)
class SingleControllerTeleOp : PracticeTeleOp<HardwareClass>({ opMode -> HardwareClass(opMode) }){
    val stateToggle = BooleanToggle(false)
    var flipState = Intake.FlipState.LOAD
    var theshold = 0.05
    override fun onLoop() {
        var leftWheels = c1.ly
        var rightWheels = c1.ry
        val dump = c1.lb
        val lift = c1.rb
        var manualThing = c1.rt - c1.lt

        if(leftWheels.absoluteValue < theshold)
            leftWheels = 0.0
        if (rightWheels.absoluteValue < theshold)
            rightWheels = 0.0
        if(manualThing.absoluteValue < theshold)
            manualThing = 0.0

        val flipUp = c1.dpu
        val flipDown = c1.dpd
        val intakeIn = c1.dpr
        val intakeOut = c1.dpl
        var intakeState = if(intakeIn) Intake.IntakeState.IN else if(intakeOut)Intake.IntakeState.OUT else Intake.IntakeState.STOP

        if(flipUp)
            flipState = Intake.FlipState.LOAD
        else if(flipDown)
            flipState = Intake.FlipState.INTAKE


        val state = if(stateToggle.update(gamepad1.left_stick_button || gamepad1.right_stick_button)) State.HANG else State.MINERAL

        robot.drive.openLoopPowerWheels(leftWheels, rightWheels)
        robot.intake.intakeState = intakeState
        robot.intake.flipState = flipState
        when(state){
            State.HANG -> {
                robot.lift.setOpenLoopPower(manualThing)
                robot.dumper.state = if(dump) Dumper.DumpState.DUMP else Dumper.DumpState.HOLD
                robot.intake.extensionState = Intake.IntakeExtensionState.STOP
            }
            State.MINERAL -> {
                robot.lift.state = if(lift) Lift.State.UP else Lift.State.DOWN
                robot.dumper.state = if(dump) Dumper.DumpState.DUMP else Dumper.DumpState.LOAD
                robot.intake.manualPowerExtension(manualThing, true)
            }
        }

        telemetry.addData("state", state)
    }
}

enum class State {
    HANG,
    MINERAL
}
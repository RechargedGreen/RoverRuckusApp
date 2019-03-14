package org.firstinspires.ftc.teamcode.iterative.lib

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.iterative.lib.commandLib.CommandSchedulerImpl
import org.firstinspires.ftc.teamcode.iterative.lib.subsystems.IterativeBotTemplate
import org.firstinspires.ftc.teamcode.iterative.lib.subsystems.SubsystemManager

abstract class IterativeOpMode<Bot : IterativeBotTemplate>(val bot: Bot, val autonomous: Boolean) : LinearOpMode() {
    val commandScheduler = CommandSchedulerImpl()

    open fun eventLoop() {
        commandScheduler.periodic()
        if (!commandScheduler.isRunningCommands())
            end()
    }

    val subsystemManager = SubsystemManager()

    override fun runOpMode() {
        telemetry.addLine("initiating bot")
        telemetry.update()
        bot.initHardware(hardwareMap, autonomous, subsystemManager)
        telemetry.addLine("bot initiated")
        telemetry.update()
        if (autonomous) {
            telemetry.addData("Status", "autoPostInit")
            telemetry.update()
            subsystemManager.autoPostInit()
            waitForStart()
            telemetry.addData("Status", "autoStart")
            telemetry.update()
            subsystemManager.autoStart()
        } else {
            telemetry.addData("Status", "teleOpPostInit")
            telemetry.update()
            subsystemManager.teleOpPostInit()
            waitingForStart()
            telemetry.addData("Status", "teleOpStart")
            telemetry.update()
            subsystemManager.teleOpStart()
        }
        onStart()
        startEventLoop()
        end()
    }

    fun end() {
        requestOpModeStop()
        onStop()
        if (autonomous)
            subsystemManager.autoEnd()
        else
            subsystemManager.teleOpEnd()
    }

    open fun onStop() {

    }

    open fun onStart() {

    }

    private fun startEventLoop() {
        while (opModeIsActive()) {
            telemetry.addData("status", "eventLoop")
            eventLoop()
            subsystemManager.update()
            telemetry.update()
        }
    }

    override fun waitForStart() {
        while (waitingForStart()) {
            tillStart()
            if (autonomous)
                subsystemManager.update()
            telemetry.update()
        }
    }

    fun waitingForStart() = !(isStarted || isStopRequested)

    open fun tillStart() {
        telemetry.addData("status", "waitForStart")
    }
}
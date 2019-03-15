package org.firstinspires.ftc.teamcode.iterative.lib

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.iterative.lib.commandLib.CommandSchedulerImpl
import org.firstinspires.ftc.teamcode.iterative.lib.subsystems.IterativeBotTemplate
import org.firstinspires.ftc.teamcode.iterative.lib.subsystems.SubsystemManager

abstract class IterativeOpMode<Bot : IterativeBotTemplate>(val bot: Bot, val autonomous: Boolean) : LinearOpMode() {
    val commandScheduler = CommandSchedulerImpl()

    @Throws(InterruptedException::class)
    open fun eventLoop() {
        commandScheduler.periodic()
        if (!commandScheduler.isRunningCommands())
            end()
    }

    val subsystemManager = SubsystemManager()

    @Throws(InterruptedException::class)
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

    @Throws(InterruptedException::class)
    fun end() {
        requestOpModeStop()
        onStop()
        if (autonomous)
            subsystemManager.autoEnd()
        else
            subsystemManager.teleOpEnd()
    }

    @Throws(InterruptedException::class)
    open fun onStop() {

    }

    @Throws(InterruptedException::class)
    open fun onStart() {

    }

    @Throws(InterruptedException::class)
    private fun startEventLoop() {
        while (opModeIsActive()) {
            telemetry.addData("status", "eventLoop")
            eventLoop()
            subsystemManager.update()
            telemetry.update()
        }
    }

    @Throws(InterruptedException::class)
    override fun waitForStart() {
        while (waitingForStart()) {
            tillStart()
            if (autonomous)
                subsystemManager.update()
            telemetry.update()
        }
    }

    @Throws(InterruptedException::class)
    fun waitingForStart() = !(isStarted || isStopRequested)

    @Throws(InterruptedException::class)
    open fun tillStart() {
        telemetry.addData("status", "waitForStart")
    }
}
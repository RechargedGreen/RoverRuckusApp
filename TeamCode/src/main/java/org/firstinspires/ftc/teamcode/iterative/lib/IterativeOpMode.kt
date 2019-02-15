package org.firstinspires.ftc.teamcode.iterative.lib

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.iterative.lib.commandLib.CommandSchedulerImpl
import org.firstinspires.ftc.teamcode.iterative.lib.subsystems.IterativeBotTemplate
import org.firstinspires.ftc.teamcode.iterative.lib.subsystems.SubsystemManager

abstract class IterativeOpMode<Bot:IterativeBotTemplate>(val bot:Bot, val autonomous:Boolean) : LinearOpMode(){
    val commandScheduler = CommandSchedulerImpl()

    open fun eventLoop(){
        if(!commandScheduler.isRunningCommands())
            end()
    }

    val subsystemManager = SubsystemManager()

    override fun runOpMode() {
        bot.initHardware(hardwareMap, autonomous, subsystemManager)
        if (autonomous) {
            subsystemManager.autoPostInit()
            waitForStart()
            subsystemManager.autoStart()
        }
        else{
            subsystemManager.teleOpPostInit()
            waitingForStart()
            subsystemManager.teleOpStart()
        }
        onStart()
        startEventLoop()
        end()
    }

    fun end(){
        requestOpModeStop()
        onStop()
        if(autonomous)
            subsystemManager.autoEnd()
        else
            subsystemManager.teleOpEnd()
    }

    open fun onStop(){

    }

    open fun onStart(){

    }

    private fun startEventLoop(){
        while (opModeIsActive()){
            telemetry.addData("status", "eventLoop")
            eventLoop()
            subsystemManager.update()
        }
    }

    override fun waitForStart(){
        while (waitingForStart()){
            tillStart()
            if(autonomous)
                subsystemManager.update()
        }
    }

    fun waitingForStart() = !(isStarted || isStopRequested)

    open fun tillStart(){
        telemetry.addData("status", "waitForStart")
    }
}
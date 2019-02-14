package org.firstinspires.ftc.teamcode.iterative.lib.commandLib

class CommandGroup(private val commands:ArrayList<Command> = ArrayList()){
    fun addCommand(command:Command){
        commands.add(command)
    }

    fun reset(){
    }

    fun update(){

    }

    fun finished(){

    }
}
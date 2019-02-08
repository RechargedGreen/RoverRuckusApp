package org.firstinspires.ftc.teamcode.mainBot.hardware

class Sensors(robot:HardwareClass){
    val lineDetector = LineDetector(robot)
    val backIntoWallDetector = BackIntoWallDetector(robot)
    val craterRimDetector = CraterRimDetector(robot)
}
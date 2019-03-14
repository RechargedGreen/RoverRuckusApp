package org.firstinspires.ftc.teamcode.mainBot.auto

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions

@Config
object SuperFastGoldSample {
    @JvmField
    var centerToMineralTicks = 0
    @JvmField
    var sideToMineralTicks = 0
    @JvmField
    var sideToDepotTicks = 0
    @JvmField
    var sideOffsetToMineral = 0
    @JvmField
    var sideOffsetToDepotIntoWall = 0

    fun doStuff(opMode:RR2Auto){
        var startAngle = RR2Auto.StartingPositions.GOLD_HANG.angle
        when(opMode.ORDER){
            SampleRandomizedPositions.CENTER, SampleRandomizedPositions.UNKNOWN -> opMode.robot.drive.deadReckonPID(centerToMineralTicks, RR2Auto.StartingPositions.GOLD_HANG.angle)
            SampleRandomizedPositions.LEFT -> {
                var degree = startAngle + sideOffsetToMineral
                opMode.robot.drive.strafeAroundLeft(degree)
                opMode.robot.drive.deadReckonPID(sideToMineralTicks, degree)

                degree = RR2Auto.CompassDirection.EAST.degrees + sideOffsetToDepotIntoWall
                opMode.robot.drive.strafeAroundRight(degree)
                opMode.robot.drive.deadReckonPID(sideToDepotTicks, degree)
            }
            SampleRandomizedPositions.RIGHT -> {
                var degree = startAngle - sideOffsetToMineral
                opMode.robot.drive.strafeAroundRight(degree)
                opMode.robot.drive.deadReckonPID(sideToMineralTicks, degree)

                degree = RR2Auto.CompassDirection.NORTH.degrees - sideOffsetToDepotIntoWall
                opMode.robot.drive.strafeAroundLeft(degree)
                opMode.robot.drive.deadReckonPID(sideToDepotTicks, degree)
            }
        }
        opMode.robot.intake.doMarker()
        goOutOfDepot(opMode)
    }


    @JvmField
    var intoWallOutOfDepotTicks = 0
    @JvmField
    var intoWallOutOfDepotOffset = 5.0

    @Throws(InterruptedException::class)
    private fun goOutOfDepot(opMode: RR2Auto){
        var angle = RR2Auto.CompassDirection.WEST.degrees - intoWallOutOfDepotOffset
        opMode.robot.drive.pidTurn(angle)
        opMode.robot.drive.deadReckonPID(intoWallOutOfDepotTicks, angle)
    }
}
package org.firstinspires.ftc.teamcode.mainBot.auto

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions

@Config
object SuperFastGoldSample {
    @JvmField
    var centerToMineralTicks: Int = 4000
    @JvmField
    var sideToMineralTicks: Int = 1750
    @JvmField
    var sideToDepotTicks: Int = 1500
    @JvmField
    var sideOffsetToMineral: Double = 33.0
    @JvmField
    var sideOffsetToDepotIntoWall: Double = 3.0

    @Throws(InterruptedException::class)
    fun doStuff(opMode: RR2Auto) {
        var startAngle = RR2Auto.StartingPositions.GOLD_HANG.angle
        val ORDER = opMode.ORDER
        when (ORDER) {
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
        if (ORDER == SampleRandomizedPositions.LEFT || ORDER == SampleRandomizedPositions.RIGHT)
            opMode.robot.drive.pidTurn(RR2Auto.CompassDirection.NORTH_EAST.degrees)
        opMode.robot.intake.doMarker()
        goOutOfDepot(opMode)
    }


    @JvmField
    var intoWallOutOfDepotTicks: Int = 1000
    @JvmField
    var intoWallOutOfDepotOffset: Double = 15.0

    @Throws(InterruptedException::class)
    private fun goOutOfDepot(opMode: RR2Auto) {
        var angle = RR2Auto.CompassDirection.WEST.degrees - intoWallOutOfDepotOffset
        opMode.robot.drive.pidTurn(angle)
        opMode.robot.drive.deadReckonPID(intoWallOutOfDepotTicks, angle)
    }
}
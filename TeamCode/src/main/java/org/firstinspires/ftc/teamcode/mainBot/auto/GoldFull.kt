package org.firstinspires.ftc.teamcode.mainBot.auto

import com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain.DiffDrive
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mainBot.hardware.DriveTerrain
import org.firstinspires.ftc.teamcode.mainBot.hardware.Intake
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups
import kotlin.math.absoluteValue

@Autonomous(group = OpModeGroups.MAIN_AUTO)
class GoldFull : RR2Auto(StartingPositions.GOLD_HANG){
    override fun postDeploy() {
        sample(SampleCollectionType.LANDER_DRIVE_FAST_TEAM_MARKER)
        park(CompassDirection.WEST.degrees)
    }
}
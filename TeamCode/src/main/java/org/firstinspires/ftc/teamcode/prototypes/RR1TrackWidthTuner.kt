package org.firstinspires.ftc.teamcode.prototypes

import com.david.rechargedkotlinlibrary.internal.roadRunner.TrackWidthCalibrationOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(group = OpModeGroups.TELE_MISC)
class RR1TrackWidthTuner():TrackWidthCalibrationOpMode<RR1Bot>({ opmode->RR1Bot(opmode) })
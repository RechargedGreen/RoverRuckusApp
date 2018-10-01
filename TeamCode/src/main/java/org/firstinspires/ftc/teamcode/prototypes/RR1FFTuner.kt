package org.firstinspires.ftc.teamcode.prototypes

import com.david.rechargedkotlinlibrary.internal.roadRunner.FeedforwardTuningOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(group = OpModeGroups.TELE_MISC)
class RR1FFTuner : FeedforwardTuningOpMode<RR1Bot>({ opMode -> RR1Bot(opMode) }, distance = 70.0)
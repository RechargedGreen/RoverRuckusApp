package org.firstinspires.ftc.teamcode.prototypes

import com.david.rechargedkotlinlibrary.internal.roadRunner.FeedforwardTuningOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class RR1FFTuner:FeedforwardTuningOpMode<RR1Bot>({ opMode -> RR1Bot(opMode) }, distance = 70.0)
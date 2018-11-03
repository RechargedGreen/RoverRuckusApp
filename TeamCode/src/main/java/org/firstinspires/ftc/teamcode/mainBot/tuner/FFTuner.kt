package org.firstinspires.ftc.teamcode.mainBot.tuner

import com.david.rechargedkotlinlibrary.internal.roadRunner.FeedforwardTuningOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mainBot.hardware.HardwareClass
import org.firstinspires.ftc.teamcode.mainBot.misc.OpModeGroups

@TeleOp(group = OpModeGroups.TELE_DIAGNOSTICS)
class FFTuner : FeedforwardTuningOpMode<HardwareClass>({ opMode -> HardwareClass(opMode) }, 40.0)
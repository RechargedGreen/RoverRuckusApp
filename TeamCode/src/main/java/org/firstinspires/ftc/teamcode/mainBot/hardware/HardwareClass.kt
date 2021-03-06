package org.firstinspires.ftc.teamcode.mainBot.hardware

import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate
import com.david.rechargedkotlinlibrary.internal.opMode.RechargedLinearOpMode
import com.david.rechargedkotlinlibrary.internal.util.AutoTransitionerKotlin
import org.firstinspires.ftc.teamcode.data.VisionConstants
import org.firstinspires.ftc.teamcode.mainBot.teleOp.PracticeJVoExtension
import org.firstinspires.ftc.teamcode.vision.MasterVision

class HardwareClass(opMode: RechargedLinearOpMode<HardwareClass>) : RobotTemplate(opMode, arrayOf("leftHub", "rightHub")) {
    val drive = DriveTerrain(this)
    val superSystem = SuperSystem(this)
    val dumper = Dumper(this)
    val intake = Intake(this)
    val lift = Lift(this)
    val vision: MasterVision = MasterVision(VisionConstants.vuforiaLocalizerParameters, hMap, opMode.isAutonomous(), MasterVision.TFLiteAlgorithm.INFER_RIGHT)
    val sensors = Sensors(this)

    @Throws(InterruptedException::class)
    override fun autoPostInit() {
        if (Static.transitionOnAutoEnd)
            AutoTransitionerKotlin.transitionOnStop(opMode, PracticeJVoExtension.NAME)
        vision.init()
        vision.enable()
    }

    @Throws(InterruptedException::class)
    override fun onPressingAutoPlay() {
        vision.shutdown()
    }
}
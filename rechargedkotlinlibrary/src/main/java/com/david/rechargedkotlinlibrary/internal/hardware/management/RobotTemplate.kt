package com.david.rechargedkotlinlibrary.internal.hardware.management

import com.david.rechargedkotlinlibrary.internal.hardware.devices.RevHub
import com.david.rechargedkotlinlibrary.internal.hardware.driveTerrain.TunableDrive
import com.david.rechargedkotlinlibrary.internal.opMode.RechargedLinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.*

/**
 * Created by David Lukens on 8/2/2018.
 */
abstract class RobotTemplate(val opMode: RechargedLinearOpMode<out RobotTemplate>, revHubNames: Array<String>) {
    val hMap: HardwareMap = opMode.hardwareMap
    val sameThreadSubsystems = HashSet<SameThreadSubsystem>()
    val thread = HardwareThread(this)

    val revHubs = LinkedList<RevHub>()

    init {
        revHubNames.forEach { revHubs.addLast(RevHub(this, it)) }
    }
    @Throws(InterruptedException::class)
    fun getHub(index: Int) = revHubs[index]
    @Throws(InterruptedException::class)
    open fun onStart() {}
    @Throws(InterruptedException::class)
    abstract fun autoPostInit()
    @Throws(InterruptedException::class)
    open fun onPressingAutoPlay() {}
    @Throws(InterruptedException::class)
    open fun onPressingTeleOpPlay() {}
    @Throws(InterruptedException::class)
    fun start() {
        thread.start()
        sameThreadSubsystems.forEach({ it.start() })
        onStart()
    }
}
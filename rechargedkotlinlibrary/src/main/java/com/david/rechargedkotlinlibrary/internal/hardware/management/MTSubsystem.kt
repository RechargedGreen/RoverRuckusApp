package com.david.rechargedkotlinlibrary.internal.hardware.management

/**
 * Created by David Lukens on 8/10/2018.
 */
interface MTSubsystem : SubSystemBase {
    @Throws(InterruptedException::class)
    fun update()
}
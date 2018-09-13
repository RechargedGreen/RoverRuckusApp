package com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors

import com.david.rechargedkotlinlibrary.internal.hardware.management.RobotTemplate

/**
 * Created by David Lukens on 8/7/2018.
 */

data class ConfigData(val robot: RobotTemplate, val hub: Int, val config: String)
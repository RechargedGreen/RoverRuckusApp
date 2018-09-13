package com.david.rechargedkotlinlibrary.internal.hardware.devices.sensors.encoders

import com.david.rechargedkotlinlibrary.internal.hardware.devices.RevHub

/**
 * Created by David Lukens on 8/8/2018.
 */
class E4T(HUB: RevHub, PORT: Int, type: E4T.Type) : Encoder(HUB, PORT, PPR = type.PPR) {
    enum class Type(val CPR: Int) {
        CPR100(100),
        CPR108(108),
        CPR112(112),
        CPR120(120),
        CPR125(125),
        CPR128(128),
        CPR200(200),
        CPR250(250),
        CPR256(256),
        CPR300(300),
        CPR360(360),
        CPR400(400),
        CPR500(500);

        val MAX_RPM = Math.min(60000, 6000000 / CPR)
        val PPR = CPR * 4
    }
}
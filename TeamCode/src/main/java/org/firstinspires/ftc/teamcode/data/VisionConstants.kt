package org.firstinspires.ftc.teamcode.data

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer

/**
 * Created by David Lukens on 11/2/2018.
 */
object VisionConstants {
    const val VUFORIA_KEY = "AXi/CxP/////AAAAGV4xMjmD2EwntmuvBtxZnj8AOji5oAG2lxjzOJIGA9IASLd1EtX7KzZ6BpH6J0FWgEcjd8O/6mWD1rvLoAZ1R3KJcxH/xss+scSbd/U8d7/cZDupryfSH7lbRv94ZmPPwduAaQOkxyZfX0Gv+IsMUtIGqTZ5WIHYpqRSHIsGQQ6nlslCi5x/NRu0tnV1t6YgX6svoenYGXpbktnCYZB5BwO7OTfw7XrMMWtqSCJrd3PZha8rgiN1VvqvdEok//H0d9Vh5pnAMa8XwMEXx0N/0V1uEGUEcQvQA+fK7zghPqxjiXBQoZxcUUGkSbNGaIfTPBEoNoOi8QzHo4N6QN1TrgLnJW9J6tgbz9xzTpnRahqU"
    val vuforiaLocalizerParameters = VuforiaLocalizer.Parameters()

    init {
        vuforiaLocalizerParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK
        vuforiaLocalizerParameters.vuforiaLicenseKey = VUFORIA_KEY
    }
}
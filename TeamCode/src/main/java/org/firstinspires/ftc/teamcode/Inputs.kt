package org.firstinspires.ftc.teamcode

import dev.nextftc.bindings.Button
import dev.nextftc.ftc.Gamepads

object Inputs {

    val intakeButton: Button get() {
        return Gamepads.gamepad1.rightTrigger.asButton { it > 0.5 }
    }


}
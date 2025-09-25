package org.firstinspires.ftc.teamcode

import com.bylazar.configurables.annotations.Configurable

@Configurable
object FakeInputs {
    @JvmField
    var dpadUp = false;
    @JvmField
    var dpadLeft = false;
    @JvmField
    var dpadRight = false

    @JvmField
    var fakeButton = false
    @JvmField
    var newAngle = 0.0

    @JvmField
    var intake = false

    @JvmField
    var shooter = false
}
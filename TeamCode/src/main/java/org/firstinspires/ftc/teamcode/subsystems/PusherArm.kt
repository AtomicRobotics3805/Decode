package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition

object PusherArm : Subsystem {

    val servo = ServoEx("servo_c0")

    val push = SetPosition(servo, 1.0)
    val retract = SetPosition(servo, 0.65)

    val pushCommand = SequentialGroup(
        push,
        Delay(0.25),
        retract
    )

}
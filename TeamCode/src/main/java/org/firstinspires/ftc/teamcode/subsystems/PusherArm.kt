package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition

object PusherArm : Subsystem {

    val servo = ServoEx("servo_c0")

    val push = SetPosition(servo, 0.0)
    val retract = SetPosition(servo, 0.25)

    val pushCommand = SequentialGroup

}
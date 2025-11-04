package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition

object PusherArm : Subsystem {

    val servo = ServoEx("servo_c0")
//    lateinit var servo: Servo

    val up = SetPosition(servo, 1.0).requires(this)
    val down = SetPosition(servo, 0.65).requires(this)
//    val up = InstantCommand { servo.position = 1.0 }
//    val down = InstantCommand { servo.position = 0.65 }

    val push get() = SequentialGroup(
        Delay(0.2),
        up,
        Delay(0.4),
        down,
        Delay(0.1)
    ).requires(this)

    override fun initialize() {
//        servo = ActiveOpMode.hardwareMap.servo["servo_c0"]
    }

}
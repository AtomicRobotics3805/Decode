package org.firstinspires.ftc.teamcode.subsystems

import androidx.core.graphics.component4
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx

object Intake : Subsystem {

    val motor = MotorEx("motor_e2").reversed()

    override fun initialize() {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    val start = InstantCommand { motor.power = 1.0 }.requires(this)
    val slowIn = InstantCommand { motor.power = 0.4 }.requires(this)
    val slowOut = InstantCommand { motor.power = -0.2 }.requires(this)
    val stop = InstantCommand { motor.power = 0.0 }.requires(this)
    val reverse = InstantCommand { motor.power = -0.4 }.requires(this)
}
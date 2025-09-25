package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx

object Intake : Subsystem {

    private val motor = MotorEx("motor_e2").reversed()

    override fun initialize() {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    val start = InstantCommand { motor.power = 1.0 }
    val stop = InstantCommand { motor.power = 0.0 }
    val reverse = InstantCommand { motor.power = -1.0 }

}
package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.builder.ControlSystemBuilder
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.RunToVelocity
import dev.nextftc.hardware.impl.MotorEx

//object Shooter : Subsystem {
//
//    val ticksPerRev = 112
//
//    val motor = MotorEx("motor_e1")
//    val encoder = MotorEx("motor_c2")
//
//
//    override fun initialize() {
//        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//    }
//
//    private val controller = controlSystem {
//        velPid(0.005, 0.0, 0.0)
//        basicFF(0.0, 0.0, 0.0)
//    }
//
//    val start = InstantCommand { motor.power = 1.0 }
//    val stop = InstantCommand { motor.power = 0.0 }
//
////    val start = RunToVelocity(controller, (5400 / 60.0) * ticksPerRev) // (desired RPM / 60) * ticks per rev
////    val stop = RunToVelocity(controller, 0.0)
//
//    override fun periodic() {
////        motor.power = controller.calculate(encoder.state)
//    }
//}

object Shooter : Subsystem {

    private val motor = MotorEx("motor_e1").reversed()

    override fun initialize() {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    val start = InstantCommand { motor.power = 1.0 }
    val stop = InstantCommand { motor.power = 0.0 }
    val reverse = InstantCommand { motor.power = -1.0 }

}

package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.ControlSystemBuilder
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.RunToVelocity
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.DecoupledMotorEx

object Shooter : Subsystem {

    val ticksPerRev = 112/4

//    val motor = MotorEx("motor_e1")
//    val encoder = MotorEx("motor_c2")

    val motor = DecoupledMotorEx("motor_e1", "motor_c2").reversed()

    override fun initialize() {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    @JvmField
    val controller = controlSystem {
        velPid(0.005, 0.0, 0.002)
        basicFF(0.0, 0.0, 0.0)
    }

//    val start = InstantCommand {  controller.goal = KineticState(0.0, (5400 / 60.0) * ticksPerRev) }
//    val stop = InstantCommand { controller.goal = KineticState() }

    val start = RunToVelocity(controller, (4000 / 60.0) * ticksPerRev, KineticState(Double.POSITIVE_INFINITY, 500.0, Double.POSITIVE_INFINITY)) // (desired RPM / 60) * ticks per rev
    val stop = RunToVelocity(controller, 0.0)

    val reverseIntake = RunToVelocity(controller, -((400 / 60.0) * ticksPerRev))

    override fun periodic() {
        motor.power = controller.calculate(motor.state)

        ActiveOpMode.telemetry.addData("Current velocity:", motor.state.velocity)
        ActiveOpMode.telemetry.addData("Target velocity:", controller.goal.velocity)
        ActiveOpMode.telemetry.addData("Shooter power", motor.power)
    }
}

//object Shooter : Subsystem {
//
//    private val motor = MotorEx("motor_e1").reversed()
//
//    override fun initialize() {
//        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//    }
//
//    val start = InstantCommand { motor.power = 1.0 }
//    val stop = InstantCommand { motor.power = 0.0 }
//    val reverse = InstantCommand { motor.power = -1.0 }
//
//}

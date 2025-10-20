package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.RunToVelocity
import org.firstinspires.ftc.teamcode.DecoupledMotorEx
import java.time.Instant

@Configurable
object Shooter : Subsystem {

    val ticksPerRev = 112/4

//    val motor = MotorEx("motor_e1")
//    val encoder = MotorEx("motor_c2")

    var controlled = true

    val motor = DecoupledMotorEx("motor_e1", "motor_c2").reversed()


    @JvmField var basicFFCoefficients = BasicFeedforwardParameters(0.001, 0.0, 0.0)
    @JvmField var velPidCoefficients = PIDCoefficients(0.018, 0.0, 0.015)

    override fun initialize() {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        controller.goal = KineticState()
    }

    @JvmField
    val controller = controlSystem {
        velPid(velPidCoefficients)
        basicFF(basicFFCoefficients)
    }

//    val start = InstantCommand {  controller.goal = KineticState(0.0, (5400 / 60.0) * ticksPerRev) }
//    val stop = InstantCommand { controller.goal = KineticState() }

    val start = RunToVelocity(controller, (2600 / 60.0) * ticksPerRev, KineticState(Double.POSITIVE_INFINITY, 500.0, Double.POSITIVE_INFINITY)).requires(this) // (desired RPM / 60) * ticks per rev
    val stop = RunToVelocity(controller, 0.0).requires(this)

    val uncontrolledStart = InstantCommand { controlled = false; motor.power = 1.0 }
    val uncontrolledStop = InstantCommand { controlled = true; motor.power = 0.0 }

    val reverseIntake = RunToVelocity(controller, -((1000 / 60.0) * ticksPerRev)).requires(this)

    override fun periodic() {
        if (controlled) {
            motor.power = controller.calculate(motor.state)
        }

        ActiveOpMode.telemetry.addData("Current velocity:", motor.state.velocity / ticksPerRev * 60.0)
        ActiveOpMode.telemetry.addData("Target velocity:", controller.goal.velocity / ticksPerRev * 60.0)
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

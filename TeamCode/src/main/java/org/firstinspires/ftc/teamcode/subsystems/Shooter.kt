package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.NullCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.hardware.controllable.RunToVelocity
import org.firstinspires.ftc.teamcode.AutoAdjustingCalc.calculatePower
import org.firstinspires.ftc.teamcode.DecoupledMotorEx
import kotlin.math.roundToInt

@Configurable
object Shooter : Subsystem {

    var shooterSpeedNoRatio = 2400

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


    var loopCount = 0
    val loopThreshold = 40
    val checkWithinToleranceForCorrectNumberOfLoops = WaitUntil {
        if (loopCount >= loopThreshold) {
            true
        } else {
            if (motor.velocity > (controller.goal.velocity-10) && motor.velocity < (controller.goal.velocity + 60)) {
                loopCount++
            } else {
                loopCount = 0
            }

            false
        }
    }

    val start: Command
        get() {
            return ParallelGroup(
                ProxyShoot { shooterSpeedNoRatio }.requires(this),
                checkWithinToleranceForCorrectNumberOfLoops
            ).requires(this)
        }

    val stop = RunToVelocity(controller, (1500 / 60.0) * ticksPerRev, KineticState(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY,
        Double.POSITIVE_INFINITY)).requires(this)

    val reverse = RunToVelocity(controller, -((1000 / 60.0) * ticksPerRev)).requires(this)


    override fun periodic() {
        if (controlled) {
            motor.power = controller.calculate(motor.state)
        }

        shooterSpeedNoRatio = calculatePower().roundToInt()

    }

    class ProxyShoot(val speed: () -> Int): Command() {
        private var command: Command = NullCommand()

        override val isDone: Boolean
            get() {
                return command.isDone
            }

        override fun start() {
            command = RunToVelocity(controller, (speed() / 60.0) * ticksPerRev, KineticState(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY,
                Double.POSITIVE_INFINITY))

            command.start()
        }

        override fun update() {
            command.update()
        }

        override fun stop(interrupted: Boolean) {
            command.stop(interrupted)
        }
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

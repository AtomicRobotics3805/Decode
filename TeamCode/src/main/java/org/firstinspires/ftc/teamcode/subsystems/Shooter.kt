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
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.commands.utility.NullCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.RunToVelocity
import org.firstinspires.ftc.teamcode.AutoAdjustingCalc.calculatePower
import org.firstinspires.ftc.teamcode.DecoupledMotorEx
import org.firstinspires.ftc.teamcode.autos.AutonomousInfo
import kotlin.math.roundToInt

@Configurable
object Shooter : Subsystem {

    var shooterSpeedNoRatio = 2400

    val ticksPerRev = 112/4

//    val motor = MotorEx("motor_e1")
//    val encoder = MotorEx("motor_c2")

    var controlled = true

    var shoot = false

    var trueStop = false

    @JvmField
    var autoShootVel = 2800.0

    var calculateVelocity = true

    val motor = DecoupledMotorEx("motor_e1", "motor_c2").reversed()


    @JvmField var basicFFCoefficients = BasicFeedforwardParameters(0.0005, 0.0, 0.055)
    @JvmField var velPidCoefficients = PIDCoefficients(0.006, 0.0, 0.018)

    override fun initialize() {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        controller.goal = KineticState()
        trueStop = false
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
            return LambdaCommand().setIsDone {
                motor.velocity > controller.goal.velocity
            }.setStart {
                trueStop = false
                calculateVelocity = true
                shoot = true
            }
        }

    val stop = LambdaCommand().setIsDone { true }.setStart {
        shoot = false
        trueStop = false
    }

    val actualStop = LambdaCommand().setIsDone { true }.setStart {
        shoot = false
        trueStop = true
    }

    val reverse = RunToVelocity(controller, -((1000 / 60.0) * ticksPerRev)).requires(this)

    override fun periodic() {
        if (controlled) {
            motor.power = controller.calculate(motor.state)
        }

        if (ActiveOpMode.opModeIsActive) {
            if (shoot && calculateVelocity && !trueStop) {
                shooterSpeedNoRatio = calculatePower().roundToInt()
                controller.goal = KineticState(0.0, (shooterSpeedNoRatio / 60.0) * ticksPerRev)
            } else if (calculateVelocity && !trueStop) {
                controller.goal = KineticState(0.0, (1500 / 60.0) * ticksPerRev)
            } else if (trueStop) {
                controller.goal = KineticState(velocity = 0.0)
            } else if (shoot) {
                controller.goal = KineticState(0.0, (autoShootVel / 60.0) * ticksPerRev, 0.0)
            }
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

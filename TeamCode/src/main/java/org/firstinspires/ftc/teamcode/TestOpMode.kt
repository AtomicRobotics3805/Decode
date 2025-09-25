package org.firstinspires.ftc.teamcode

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import dev.nextftc.bindings.button
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.Direction
import dev.nextftc.hardware.impl.IMUEx
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.PusherArm
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer



@TeleOp
class TestOpMode : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(Spindexer, Intake, Shooter),
            BulkReadComponent,
            BindingsComponent
        )
    }

    private val frontLeftMotor = MotorEx("front_left").brakeMode().reversed()
    private val frontRightMotor = MotorEx("front_right").brakeMode()
    private val backLeftMotor = MotorEx("back_left").brakeMode().reversed()
    private val backRightMotor = MotorEx("back_right").brakeMode()
    private val imu = IMUEx("imu", Direction.UP, Direction.FORWARD).zeroed()

    val rightTrigger = button { gamepad1.right_trigger > 0.2 }
    val leftTrigger = button { gamepad1.left_trigger > 0.2 }
    val dpadLeft = button { gamepad1.dpad_left }
    val dpadUp = button { gamepad1.dpad_up }
    val dpadRight = button { gamepad1.dpad_right }
    val dpadDown = button { gamepad1.dpad_down }
    val leftBumper = button { gamepad1.left_bumper }
    val rightBumper = button { gamepad1.right_bumper }


    override fun onStartButtonPressed() {
        val driverControlled = MecanumDriverControlled(
            frontLeftMotor,
            frontRightMotor,
            backLeftMotor,
            backRightMotor,
            Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            Gamepads.gamepad1.rightStickX
        )
        driverControlled()
        dpadUp.whenBecomesTrue { Spindexer.setAngle(0.rad).invoke() }
        dpadLeft.whenBecomesTrue { Spindexer.setAngle(120.deg).invoke() }
        dpadRight.whenBecomesTrue {
            Spindexer.setAngle((-120).deg).invoke()
            dpadDown.whenBecomesTrue { Spindexer.advanceToTravelPosition.invoke() }
            rightTrigger.whenBecomesTrue { Intake.start() }
            rightTrigger.whenBecomesFalse { Intake.stop() }
            leftTrigger.whenBecomesTrue { Shooter.start() }
            leftTrigger.whenBecomesFalse { Shooter.stop() }
            leftBumper.whenBecomesTrue { }
        }

    }
}
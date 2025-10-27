package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.subsystems.SpindexerSensor
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.control.KineticState
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.FieldCentric
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
            SubsystemComponent(Spindexer, Intake, Shooter, PusherArm, SpindexerSensor/*, ZeroSensor, LimeLight*/),
            BulkReadComponent,
            BindingsComponent,
//            PedroComponent(Constants::createFollower)
        )
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
    }

    private val frontLeftMotor = MotorEx("motor_c1").brakeMode().reversed()
    private val frontRightMotor = MotorEx("motor_c2").brakeMode()
    private val backLeftMotor = MotorEx("motor_c0").brakeMode().reversed()
    private val backRightMotor = MotorEx("motor_c3").brakeMode()
    private val imu = IMUEx("imu", Direction.LEFT, Direction.UP).zeroed()

    override fun onStartButtonPressed() {

//        Gamepads.gamepad1.a.whenBecomesTrue { Spindexer.spinToGreen() }
//        Gamepads.gamepad1.x.whenBecomesTrue { SequentialGroup(PusherArm.push, Spindexer.spinToPurple)() }
//        Gamepads.gamepad1.b.whenBecomesTrue { Spindexer.spinToIntake() }
//        Gamepads.gamepad1.y.whenBecomesTrue { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY }

        PusherArm.down()
        val driverControlled = MecanumDriverControlled(
            frontLeftMotor,
            frontRightMotor,
            backLeftMotor,
            backRightMotor,
            -Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            Gamepads.gamepad1.rightStickX,
            FieldCentric(imu)
        )
        driverControlled()
//        dpadUp.whenBecomesTrue { Spindexer.setAngle(0.rad).invoke() }
//        dpadLeft.whenBecomesTrue { Spindexer.setAngle(120.deg).invoke() }
//        dpadRight.whenBecomesTrue { Spindexer.setAngle((-120).deg).invoke() }

        Gamepads.gamepad1.rightTrigger.asButton { it > 0.5 } whenBecomesTrue { Routines.intake() }



        Gamepads.gamepad1.a.whenBecomesTrue { Spindexer.spinToIntake() }

        Gamepads.gamepad1.x.whenBecomesTrue { if (!Spindexer.currentStatus.onTop) Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE }
        Gamepads.gamepad1.b.whenBecomesTrue { if (!Spindexer.currentStatus.onTop) Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN }
        Gamepads.gamepad1.y.whenBecomesTrue { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY }

        Gamepads.gamepad1.dpadUp

//        Gamepads.gamepad1.y.whenBecomesTrue { Spindexer.setAngle((60).deg).schedule() }
//        Gamepads.gamepad1.b.whenBecomesTrue { Spindexer.setAngle((180).deg).schedule() }

        Gamepads.gamepad1.leftTrigger.asButton { it > 0.5 }.whenBecomesTrue { Intake.reverse() }.whenBecomesFalse { Intake.slowOut() }
        Gamepads.gamepad1.rightBumper.whenBecomesTrue { Routines.shoot() }

        Gamepads.gamepad1.start.whenBecomesTrue { Spindexer.spinToPurple() }
        Gamepads.gamepad1.back.whenBecomesTrue { Spindexer.spinToGreen() }

        Gamepads.gamepad1.rightStickButton.whenBecomesTrue { imu.zero() }

        Gamepads.gamepad1.leftBumper.whenBecomesTrue { Shooter.reverseIntake() }.whenBecomesFalse { Shooter.stop() }
//        Gamepads.gamepad1.leftStickButton.whenBecomesTrue { Shooter.uncontrolledStart() }.whenBecomesFalse { Shooter.uncontrolledStop() }

        Gamepads.gamepad1.dpadLeft.whenTrue { Spindexer.zeroClockwise() }.whenBecomesFalse { Spindexer.endZero() }
        Gamepads.gamepad1.dpadRight.whenTrue { Spindexer.zeroCounterClockwise() }.whenBecomesFalse { Spindexer.endZero() }

        Gamepads.gamepad1.dpadUp.whenTrue { Routines.motifShoot() }
    }

    override fun onStop() {
//        LimeLight.stop()
        Spindexer.controller.goal = KineticState()
        Shooter.controller.goal = KineticState()
    }
}
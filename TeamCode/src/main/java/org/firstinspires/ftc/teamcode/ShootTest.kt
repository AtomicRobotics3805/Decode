package org.firstinspires.ftc.teamcode

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.bindings.button
import dev.nextftc.control.KineticState
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
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
@Disabled
class ShootTest : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(Shooter, PusherArm),
            BulkReadComponent,
            BindingsComponent
        )
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
    }
//
//    private val frontLeftMotor = MotorEx("motor_c1").brakeMode().reversed()
//    private val frontRightMotor = MotorEx("motor_c2").brakeMode()
//    private val backLeftMotor = MotorEx("motor_c0").brakeMode().reversed()
//    private val backRightMotor = MotorEx("motor_c3").brakeMode()
//    private val imu = IMUEx("imu", Direction.LEFT, Direction.UP)//.zeroed()
//
//    val rightTrigger = button { gamepad1.right_trigger > 0.2 }
//    val leftTrigger = button { gamepad1.left_trigger > 0.2 }
//    val dpadLeft = button { gamepad1.dpad_left }
//    val dpadUp = button { gamepad1.dpad_up }
//    val dpadRight = button { gamepad1.dpad_right }
//    val dpadDown = button { gamepad1.dpad_down }
//    val leftBumper = button { gamepad1.left_bumper }
//    val rightBumper = button { gamepad1.right_bumper }
//    val startButton = button { gamepad1.start }

    override fun onStartButtonPressed() {
        PusherArm.down()
//        val driverControlled = MecanumDriverControlled(
//            frontLeftMotor,
//            frontRightMotor,
//            backLeftMotor,
//            backRightMotor,
//            -Gamepads.gamepad1.leftStickY,
//            Gamepads.gamepad1.leftStickX,
//            Gamepads.gamepad1.rightStickX,
//            FieldCentric(imu)
//        )
//        driverControlled()
//        dpadUp.whenBecomesTrue { Spindexer.setAngle(0.rad).invoke() }
//        dpadLeft.whenBecomesTrue { Spindexer.setAngle(120.deg).invoke() }
//        dpadRight.whenBecomesTrue { Spindexer.setAngle((-120).deg).invoke() }
//
//
//        dpadDown.whenBecomesTrue { Spindexer.enableTravel() }.whenBecomesFalse { Spindexer.disableTravel() }
//        Gamepads.gamepad1.a.whenBecomesTrue { Spindexer.advanceToIntake() }
//
//        Gamepads.gamepad1.dpadLeft.whenBecomesTrue { Spindexer.setAngle((-120).deg).schedule() }
//        Gamepads.gamepad1.dpadUp.whenBecomesTrue { Spindexer.setAngle((0).deg).schedule() }
//        Gamepads.gamepad1.dpadRight.whenBecomesTrue { Spindexer.setAngle((120).deg).schedule() }
//
//        Gamepads.gamepad1.x.whenBecomesTrue { if (!Spindexer.currentStatus.onTop) Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE }
//        Gamepads.gamepad1.b.whenBecomesTrue { if (!Spindexer.currentStatus.onTop) Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN }
//        Gamepads.gamepad1.y.whenBecomesTrue { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY }
//
////        Gamepads.gamepad1.y.whenBecomesTrue { Spindexer.setAngle((60).deg).schedule() }
////        Gamepads.gamepad1.b.whenBecomesTrue { Spindexer.setAngle((180).deg).schedule() }
//
//        rightTrigger.whenBecomesTrue { Intake.start() }
//        rightTrigger.whenBecomesFalse { Intake.stop() }
//
//        leftTrigger.whenBecomesTrue { Intake.reverse() }.whenBecomesFalse { Intake.stop() }
//        rightBumper.whenBecomesTrue { Routines.shoot() }
//
//        startButton.whenBecomesTrue { Spindexer.advanceToPurple() }
//        Gamepads.gamepad1.back.whenBecomesTrue { Spindexer.advanceToGreen() }
//
//        Gamepads.gamepad1.leftBumper.whenBecomesTrue { Shooter.reverseIntake() }.whenBecomesFalse { Shooter.stop() }
        Gamepads.gamepad1.leftBumper.whenBecomesTrue { Shooter.start() }.whenBecomesFalse { Shooter.stop() }
        Gamepads.gamepad1.a.whenBecomesTrue { PusherArm.up() }.whenBecomesFalse { PusherArm.down() }
    }

    override fun onStop() {
//        Spindexer.controller.goal = KineticState()
        Shooter.controller.goal = KineticState()
    }
}
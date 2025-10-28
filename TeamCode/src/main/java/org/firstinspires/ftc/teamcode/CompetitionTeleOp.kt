package org.firstinspires.ftc.teamcode

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.FieldCentric
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.Direction
import dev.nextftc.hardware.impl.IMUEx
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.LimeLight
import org.firstinspires.ftc.teamcode.subsystems.PusherArm
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSensor
import org.firstinspires.ftc.teamcode.subsystems.ZeroSensor

@TeleOp(name = "Competition TeleOp")
class CompetitionTeleOp : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(Spindexer, Intake, Shooter, PusherArm, SpindexerSensor, LimeLight,
                ZeroSensor),
            BulkReadComponent,
//            PedroComponent(Constants::createFollower),
            BindingsComponent
        )

        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
    }

    private val frontLeftMotor = MotorEx("motor_c1").brakeMode().reversed()
    private val frontRightMotor = MotorEx("motor_c2").brakeMode()
    private val backLeftMotor = MotorEx("motor_c0").brakeMode().reversed()
    private val backRightMotor = MotorEx("motor_c3").brakeMode()
    private val imu = IMUEx("imu", Direction.LEFT, Direction.UP).zeroed()

    override fun onStartButtonPressed() {
        PusherArm.down()

        //region Gamepad bindings

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


        //region Driver


        Gamepads.gamepad1.rightStickButton whenBecomesTrue { imu.zero() }
        Gamepads.gamepad1.rightBumper whenBecomesTrue {
            driverControlled.scalar = 0.5
        } whenBecomesFalse {
            driverControlled.scalar = 1.0
        } // TODO: Add autoalign and make this enable auto alignment

        Gamepads.gamepad1.rightTrigger.asButton { it > 0.5 } whenBecomesTrue Routines.intake::schedule whenBecomesFalse Intake.slowOut::schedule

        Gamepads.gamepad1.leftBumper whenBecomesTrue Routines.shoot::schedule
        Gamepads.gamepad1.leftTrigger.asButton { it > 0.5 } whenBecomesTrue Routines.motifShoot whenBecomesFalse Routines.stopShoot

        Gamepads.gamepad1.back whenTrue Spindexer.zeroCounterClockwise::schedule whenBecomesFalse Spindexer.endZero::schedule
        Gamepads.gamepad1.start whenTrue Spindexer.zeroClockwise::schedule whenBecomesFalse Spindexer.endZero::schedule

        Gamepads.gamepad1.x.whenBecomesTrue {
            if (!Spindexer.currentStatus.onTop) Spindexer.slots[Spindexer.currentStatus.id] =
                Spindexer.SpindexerSlotStatus.PURPLE
        }
        Gamepads.gamepad1.a.whenBecomesTrue {
            if (!Spindexer.currentStatus.onTop) Spindexer.slots[Spindexer.currentStatus.id] =
                Spindexer.SpindexerSlotStatus.GREEN
        }
        Gamepads.gamepad1.y whenBecomesTrue {
            Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY
        }

        Gamepads.gamepad1.dpadUp whenBecomesTrue Spindexer.enableTraveling::schedule
        Gamepads.gamepad1.dpadLeft whenBecomesTrue Spindexer.spinToGreen::schedule
        Gamepads.gamepad1.dpadRight whenBecomesTrue Spindexer.spinToPurple::schedule
        Gamepads.gamepad1.dpadDown whenBecomesTrue Spindexer.spinToIntake::schedule


        //endregion

        //endregion
    }


}
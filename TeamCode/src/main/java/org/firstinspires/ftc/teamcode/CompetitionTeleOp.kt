package org.firstinspires.ftc.teamcode

import com.bylazar.graph.PanelsGraph
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.RobotLog
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.FieldCentric
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.Direction
import dev.nextftc.hardware.impl.IMUEx
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.LimeLight
import org.firstinspires.ftc.teamcode.subsystems.LimeLight.matchMotif
import org.firstinspires.ftc.teamcode.subsystems.PusherArm
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Shooter.controller
import org.firstinspires.ftc.teamcode.subsystems.Shooter.motor
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ticksPerRev
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.subsystems.Spindexer.ticksToAngle
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSensor
import org.firstinspires.ftc.teamcode.subsystems.ZeroSensor

@TeleOp(name = "Competition TeleOp")
class CompetitionTeleOp : NextFTCOpMode() {
    var graphManager = PanelsGraph.manager
    init {
        addComponents(
            SubsystemComponent(Spindexer, Intake, Shooter, PusherArm, SpindexerSensor, /*LimeLight,*/
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

        graphManager = PanelsGraph.manager

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

    override fun onUpdate() {
        ActiveOpMode.telemetry.addData("Motor Amp: Left Front Drive: ", frontLeftMotor.motor.getCurrent(CurrentUnit.AMPS))
        ActiveOpMode.telemetry.addData("Motor Amp: Left Back Drive: ", backLeftMotor.motor.getCurrent(CurrentUnit.AMPS))
        ActiveOpMode.telemetry.addData("Motor Amp: Right Front Drive", frontRightMotor.motor.getCurrent(CurrentUnit.AMPS))
        ActiveOpMode.telemetry.addData("Motor Amp: Right Back Drive", backRightMotor.motor.getCurrent(CurrentUnit.AMPS))

        ActiveOpMode.telemetry.addData("Motor Amp: Intake Motor", Intake.motor.motor.getCurrent(CurrentUnit.AMPS))
        ActiveOpMode.telemetry.addData("Motor Amp: Spindexer Motor", Spindexer.motor.motor.getCurrent(CurrentUnit.AMPS))
        ActiveOpMode.telemetry.addData("Motor Amp: Shooter Motor", Shooter.motor.motor.getCurrent(CurrentUnit.AMPS))

        ActiveOpMode.telemetry.addData("Current velocity:", motor.state.velocity / ticksPerRev * 60.0)
        ActiveOpMode.telemetry.addData("Target velocity:", controller.goal.velocity / ticksPerRev * 60.0)
        ActiveOpMode.telemetry.addData("Shooter power", motor.power)

        ActiveOpMode.telemetry.addData("Motif:", matchMotif.toString())

        ActiveOpMode.telemetry.addData("Raw Encoder", Spindexer.motor.state.position)
        ActiveOpMode.telemetry.addData("Angle", ticksToAngle(Spindexer.motor.state.position).normalized.inDeg)
        ActiveOpMode.telemetry.addData("Spindexer goal", Spindexer.controller.goal.position.rad.inDeg)
        ActiveOpMode.telemetry.addData("Spindexer slots", "0:["+ Spindexer.slots[0]+"], 1:["+ Spindexer.slots[1]+"], 2:["+ Spindexer.slots[2]+"]")

        ActiveOpMode.telemetry.update()
    }


}
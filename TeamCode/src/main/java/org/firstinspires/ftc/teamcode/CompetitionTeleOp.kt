package org.firstinspires.ftc.teamcode

import android.icu.number.Scale
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.field.Style
import com.bylazar.graph.PanelsGraph
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.RobotLog
import dev.nextftc.bindings.BindingManager
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.AngleType
import dev.nextftc.control.feedback.AngularFeedback
import dev.nextftc.control.feedback.FeedbackType
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedback.PIDElement
import dev.nextftc.control.feedforward.BasicFeedforward
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.ParallelRaceGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.m
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.fateweaver.FateComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.components.LoopTimeComponent
import dev.nextftc.hardware.driving.FieldCentric
import dev.nextftc.hardware.driving.MecanumDriverControlled
import org.firstinspires.ftc.teamcode.subsystems.Direction
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.AutoAdjustingCalc.goalPos
import org.firstinspires.ftc.teamcode.autos.AutonomousInfo
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Gyro
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.LimeLight
import org.firstinspires.ftc.teamcode.subsystems.LimeLight.matchMotif
import org.firstinspires.ftc.teamcode.subsystems.PusherArm
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.subsystems.Spindexer.ticksToAngle
import java.util.function.Supplier
import kotlin.math.abs
import kotlin.math.sqrt
import kotlin.time.ComparableTimeMark
import kotlin.time.DurationUnit
import kotlin.time.TimeSource.Monotonic.markNow


@Configurable
@TeleOp(name = "Competition TeleOp")
class CompetitionTeleOp : NextFTCOpMode() {
    var graphManager = PanelsGraph.manager
    init {
        addComponents(
            SubsystemComponent(Spindexer, Intake, Shooter, PusherArm, /*SpindexerSensor,*/ LimeLight
                /*ZeroSensor*/),
            BulkReadComponent,
            PedroComponent(Constants::createFollower),
            BindingsComponent,
//            FateComponent,
            LoopTimeComponent()
        )
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
    }

    val debugTelemetry = false




    var imuOffset = 0.0.deg.inRad

    companion object {
        @JvmField
        var mainCoefficients = PIDCoefficients(-1.25, 0.0, 0.05)

        @JvmField
        var secondaryCoefficients = PIDCoefficients(-1.0, 0.0, 0.2)

        @JvmField
        var pidSwitch = 30.0


        @JvmField
        val mainController = controlSystem {
            feedback(
                AngularFeedback(
                    AngleType.RADIANS,
                    PIDElement(
                        FeedbackType.POSITION,
                        mainCoefficients
                    )
                )
            )
        }

        @JvmField
        val secondaryController = controlSystem {
            feedback(
                AngularFeedback(
                    AngleType.RADIANS,
                    PIDElement(
                        FeedbackType.POSITION,
                        secondaryCoefficients
                    )
                )
            )
        }

        fun calculateHeadingPID(): Double {
            if (abs(AutoAdjustingCalc.calculateAimAngle() - PedroComponent.follower.heading) < pidSwitch.deg.inRad) {
                secondaryController.goal = KineticState(AutoAdjustingCalc.calculateAimAngle())

                return secondaryController.calculate(KineticState(PedroComponent.follower.heading))
            } else {
                mainController.goal = KineticState(AutoAdjustingCalc.calculateAimAngle())

                return mainController.calculate(KineticState(PedroComponent.follower.heading))
            }
        }

        val autoAimPID = PIDJoystickBlend(Gamepads.gamepad1.rightStickX::get, ::calculateHeadingPID)

        val imu = Gyro("imu", Direction.LEFT, Direction.UP).zeroed()
    }

    private val frontLeftMotor = MotorEx("motor_c1").brakeMode()
    private val frontRightMotor = MotorEx("motor_c2").brakeMode()
    private val backLeftMotor = MotorEx("motor_c0").brakeMode()
    private val backRightMotor = MotorEx("motor_c3").brakeMode()

    private lateinit var startTime: ComparableTimeMark

    var iterations = 0

    override fun onInit() {
        Drawing.init()
    }

    override fun onStartButtonPressed() {
        LimeLight.autoRelocalize = true
        LimeLight.obeliskMode = false
        LimeLight.checkPipeline()
        AutonomousInfo.autoRunning = false
        PedroComponent.follower.pose = AutonomousInfo.autoEndPos
        PusherArm.down()
        Routines.setMotifSelection()

        startTime = markNow()
        iterations = 0

        imu.offset = (-PedroComponent.follower.heading).rad

//        imu.zero()
//        imuOffset = AutonomousInfo.finalHeading

        graphManager = PanelsGraph.manager

        //region Gamepad bindings

        val driverControlled = MecanumDriverControlled(
            frontLeftMotor,
            frontRightMotor,
            backLeftMotor,
            backRightMotor,
            -Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            autoAimPID,
            FieldCentric(imu)
        )
        driverControlled()


        //region Driver


        Gamepads.gamepad1.rightStickButton whenBecomesTrue {
            imu.zero()

            if (AutonomousInfo.redAuto) {
                PedroComponent.follower.pose = PedroComponent.follower.pose.withHeading(0.0)
            } else {
                PedroComponent.follower.pose = PedroComponent.follower.pose.withHeading(180.deg.inRad)
            }
        }

        Gamepads.gamepad1.leftTrigger.asButton { it >= 0.1 } whenBecomesTrue {
            Spindexer.spinToIntake()
            Intake.intakeModifier = -1.0
        } whenTrue {
            Intake.motor.power = ((Gamepads.gamepad1.leftTrigger.get()/2)+0.5) * Intake.intakeModifier
        } whenBecomesFalse {
            Intake.slowOut()
        }

        Gamepads.gamepad1.rightBumper whenBecomesTrue {
                driverControlled.scalar = 0.3
            } whenBecomesFalse {
                driverControlled.scalar = 1.0
            }



        Gamepads.gamepad1.rightTrigger.asButton { it >= 0.1 } whenBecomesTrue {
            Spindexer.spinToIntake()
            Intake.intakeModifier = 1.0
        } whenTrue {
            Intake.motor.power = ((Gamepads.gamepad1.rightTrigger.get()/2)+0.5) * Intake.intakeModifier
        } whenBecomesFalse {
            Intake.slowOut()
        }

        Gamepads.gamepad1.leftBumper whenBecomesTrue({
            Routines.teleOpMotifShoot()
            autoAimPID.  usePID = true
        }) whenBecomesFalse({
                Shooter.stop()
                PusherArm.down()
                autoAimPID.usePID = false
            }
        )

        Gamepads.gamepad1.back whenTrue Spindexer.zeroCounterClockwise whenBecomesFalse Spindexer.endZero
        Gamepads.gamepad1.start whenTrue Spindexer.zeroClockwise whenBecomesFalse Spindexer.endZero

        Gamepads.gamepad1.x.whenBecomesTrue {
            Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE
            Spindexer.spinToIntake()
        }
        Gamepads.gamepad1.a.whenBecomesTrue {
            Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN
            Spindexer.spinToIntake()
        }

        Gamepads.gamepad1.y.whenBecomesTrue {
            Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY
        }

        Gamepads.gamepad1.dpadUp whenBecomesTrue Routines.shoot
        Gamepads.gamepad1.dpadLeft whenBecomesTrue Spindexer.spinToGreen
        Gamepads.gamepad1.dpadRight whenBecomesTrue ParallelGroup(
            Shooter.stop,
            Intake.slowOut,
            PusherArm.down
        ).requires(Spindexer)

        Gamepads.gamepad1.dpadDown whenBecomesTrue Routines.motifShoot whenBecomesFalse {
            Shooter.stop()
            PusherArm.down()
        }
        Gamepads.gamepad1.b whenBecomesTrue { Spindexer.spinToLast() }

//        Gamepads.gamepad1.leftStickButton whenBecomesTrue LimeLight.detectMotif
        Gamepads.gamepad1.leftStickButton whenBecomesTrue SequentialGroupLocal(
            InstantCommand {
                LimeLight.obeliskMode = true
                LimeLight.checkPipeline()
            },
            Delay(0.25),
            LimeLight.detectMotif,
            InstantCommand {
                LimeLight.obeliskMode = false
                LimeLight.checkPipeline()
            })



        //endregion

        //region Backup

        Gamepads.gamepad2.dpadLeft whenBecomesTrue Spindexer.spinToSlotZero
        Gamepads.gamepad2.dpadUp whenBecomesTrue Spindexer.spinToSlotOne
        Gamepads.gamepad2.dpadRight whenBecomesTrue Spindexer.spinToSlotTwo

        Gamepads.gamepad2.back whenBecomesTrue { AutonomousInfo.redAuto = !AutonomousInfo.redAuto }

//        Gamepads.gamepad2.dpadUp whenBecomesTrue { Shooter.shooterSpeedNoRatio += 50 }
//        Gamepads.gamepad2.dpadDown whenBecomesTrue { Shooter.shooterSpeedNoRatio -= 50 }

        Gamepads.gamepad2.a.whenBecomesTrue {
            Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN
        }
        Gamepads.gamepad2.x.whenBecomesTrue {
            Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE
        }
        Gamepads.gamepad2.b.whenBecomesTrue {
            Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY
        }

        Gamepads.gamepad2.rightBumper whenBecomesTrue {
            autoAimPID.usePID = true
        } whenBecomesFalse {
            autoAimPID.usePID = false
        }

        Gamepads.gamepad2.rightStickButton.whenBecomesTrue {
            if (AutonomousInfo.redAuto) {
                PedroComponent.follower.pose = PedroComponent.follower.pose.withHeading(0.0)
            } else {
                PedroComponent.follower.pose = PedroComponent.follower.pose.withHeading(180.deg.inRad)
            }
        }


        //endregion

        //endregion
    }

    override fun onUpdate() {
        Drawing.drawDebug(PedroComponent.Companion.follower)

        iterations++

        ActiveOpMode.telemetry.addData("Average loop time", (markNow() - startTime).toDouble(
            DurationUnit.MILLISECONDS) / iterations)

        ActiveOpMode.telemetry.addData("tX", LimeLight.getTX())

        RobotLog.d("Motor Amp: Left Front Drive: " + frontLeftMotor.motor.getCurrent(CurrentUnit.AMPS).toString())
        RobotLog.d("Motor Amp: Left Back Drive: " + backLeftMotor.motor.getCurrent(CurrentUnit.AMPS).toString())
        RobotLog.d("Motor Amp: Right Front Drive: " + frontRightMotor.motor.getCurrent(CurrentUnit.AMPS).toString())
        RobotLog.d("Motor Amp: Right Back Drive: " + backRightMotor.motor.getCurrent(CurrentUnit.AMPS).toString())

        RobotLog.d("Motor Amp: Intake Motor: " + Intake.motor.motor.getCurrent(CurrentUnit.AMPS).toString())
        RobotLog.d("Motor Amp: Spindexer Motor: " + Spindexer.motor.motor.getCurrent(CurrentUnit.AMPS).toString())
        RobotLog.d("Motor Amp: Shooter Motor: " + Shooter.motor.motor.getCurrent(CurrentUnit.AMPS).toString())


        if (debugTelemetry) {
            ActiveOpMode.telemetry.addData(
                "Current velocity:",
                Shooter.motor.state.velocity / Shooter.ticksPerRev * 60.0
            )
            ActiveOpMode.telemetry.addData(
                "Target velocity:",
                Shooter.controller.goal.velocity / Shooter.ticksPerRev * 60.0
            )
            ActiveOpMode.telemetry.addData("Shooter power:", Shooter.motor.power)

            ActiveOpMode.telemetry.addData("Motif:", matchMotif.toString())

            ActiveOpMode.telemetry.addData("Raw Encoder", Spindexer.motor.state.position)
            ActiveOpMode.telemetry.addData(
                "Angle",
                ticksToAngle(Spindexer.motor.state.position).normalized.inDeg
            )
            ActiveOpMode.telemetry.addData(
                "Spindexer goal",
                Spindexer.controller.goal.position.rad.inDeg
            )
            ActiveOpMode.telemetry.addData(
                "Spindexer slots",
                "0:[" + Spindexer.slots[0] + "], 1:[" + Spindexer.slots[1] + "], 2:[" + Spindexer.slots[2] + "]"
            )
            ActiveOpMode.telemetry.addData("Spindexer status", Spindexer.currentStatus)

            ActiveOpMode.telemetry.addData("Pose", PedroComponent.follower.pose)

            ActiveOpMode.telemetry.addLine("=+=+=+=+=+=+=+=+=+=")

            if (AutonomousInfo.redAuto) {
                ActiveOpMode.telemetry.addData(
                    "Distance",
                    if (!LimeLight.ll.latestResult.fiducialResults.any { it.fiducialId == if (AutonomousInfo.redAuto) 24 else 20 } ) {
                        if (AutonomousInfo.redAuto) {
                            sqrt(((goalPos.mirror().x - PedroComponent.follower.pose.x) * (goalPos.mirror().x - PedroComponent.follower.pose.x)) + ((goalPos.mirror().y - PedroComponent.follower.pose.y) * (goalPos.mirror().y - PedroComponent.follower.pose.y)))
                        } else {
                            sqrt(((goalPos.x - PedroComponent.follower.pose.x) * (goalPos.x - PedroComponent.follower.pose.x)) + ((goalPos.y - PedroComponent.follower.pose.y) * (goalPos.y - PedroComponent.follower.pose.y)))
                        }
                    } else {
                        abs(LimeLight.ll.latestResult.fiducialResults[0].robotPoseTargetSpace.position.z.m.inIn)
                    }
                )
            } else {
                ActiveOpMode.telemetry.addData(
                    "Distance",
                    sqrt(((goalPos.x - PedroComponent.follower.pose.x) * (goalPos.x - PedroComponent.follower.pose.x)) + ((goalPos.y - PedroComponent.follower.pose.y) * (goalPos.y - PedroComponent.follower.pose.y)))
                )
            }
            ActiveOpMode.telemetry.addData("Shooter target", Shooter.shooterSpeedNoRatio)
        } else {
            ActiveOpMode.telemetry.addData("Red Side", AutonomousInfo.redAuto)

            ActiveOpMode.telemetry.addData("Motif:", matchMotif.toString())

            ActiveOpMode.telemetry.addData(
                "Spindexer slots",
                "0:[" + Spindexer.slots[0] + "], 1:[" + Spindexer.slots[1] + "], 2:[" + Spindexer.slots[2] + "]"
            )

            ActiveOpMode.telemetry.addData("Spindexer status", Spindexer.currentStatus)

            ActiveOpMode.telemetry.addData(
                "Distance",
                if (!LimeLight.ll.latestResult.fiducialResults.any { it.fiducialId == if (AutonomousInfo.redAuto) 24 else 20 } ) {
                    if (AutonomousInfo.redAuto) {
                        sqrt(((goalPos.mirror().x - PedroComponent.follower.pose.x) * (goalPos.mirror().x - PedroComponent.follower.pose.x)) + ((goalPos.mirror().y - PedroComponent.follower.pose.y) * (goalPos.mirror().y - PedroComponent.follower.pose.y)))
                    } else {
                        sqrt(((goalPos.x - PedroComponent.follower.pose.x) * (goalPos.x - PedroComponent.follower.pose.x)) + ((goalPos.y - PedroComponent.follower.pose.y) * (goalPos.y - PedroComponent.follower.pose.y)))
                    }
                } else {
                    abs(LimeLight.ll.latestResult.fiducialResults[0].robotPoseTargetSpace.position.z.m.inIn)
                }
            )
            ActiveOpMode.telemetry.addData(
                "Using LL dist",
                LimeLight.ll.latestResult.fiducialResults.any { it.fiducialId == if (AutonomousInfo.redAuto) 24 else 20 }
            )

            telemetry.addData("Auto align", AutoAdjustingCalc.pose)

            ActiveOpMode.telemetry.addData("Auto Align", autoAimPID.usePID)
        }

        ActiveOpMode.telemetry.update()
    }


}
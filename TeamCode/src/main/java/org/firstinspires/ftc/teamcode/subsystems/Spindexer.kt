package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.AngleType
import dev.nextftc.control.feedback.AngularFeedback
import dev.nextftc.control.feedback.FeedbackType
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedback.PIDElement
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.conditionals.IfElseCommand
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.commands.utility.NullCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.Gamepads
import dev.nextftc.hardware.controllable.RunToPosition
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.DecoupledMotorEx
import org.firstinspires.ftc.teamcode.subsystems.Spindexer.slots
import kotlin.math.PI
import kotlin.math.abs


@Configurable
object Spindexer : Subsystem {

    enum class SpindexerSlotStatus {
        EMPTY,
        PURPLE,
        GREEN
    }

    enum class SpindexerStatus(
        val onTop: Boolean,
        val id: Int,
        val angle: Angle
    ) {
        TOP_0(true, 0, 0.deg),
        TOP_1(true, 1, 120.deg),
        TOP_2(true, 2,(-120).deg),
        BOTTOM_0(false, 0, 180.deg),
        BOTTOM_1(false,1, (-60).deg),
        BOTTOM_2(false,2, 60.deg)
    }

    var slots : Array<SpindexerSlotStatus> = arrayOf(SpindexerSlotStatus.GREEN, SpindexerSlotStatus.PURPLE, SpindexerSlotStatus.PURPLE)

    @JvmField
    var currentStatus = SpindexerStatus.TOP_0

    @JvmField
    var traveling = false

    var controllerDisabled = false

    var empty = false

    const val ticksPerRev = 1425.1

    @JvmField
    var wiggleTolerance = (ticksPerRev * (3).deg.inRad) / (2 * PI)

    val motor = DecoupledMotorEx("motor_e0", "motor_e0")

    override fun initialize() {
        motor.encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        controller.goal = KineticState(0.0)
        currentStatus = SpindexerStatus.TOP_0
        slots = arrayOf(SpindexerSlotStatus.GREEN, SpindexerSlotStatus.PURPLE, SpindexerSlotStatus.PURPLE)
//        motor.zero()
    }

    @JvmField
    var coefficients = PIDCoefficients(0.5, 0.0, 0.0001)

    @JvmField
    val controller = controlSystem {
        posFilter {
            it.custom { ticks ->
                ticksToAngle(ticks).normalized.inRad
            }
        }

        feedback(
            AngularFeedback(
                AngleType.RADIANS,
                PIDElement(
                    FeedbackType.POSITION,
                    coefficients
                )
            )
        )
    }

    override fun periodic() {
        controller.goal = KineticState(currentStatus.angle.inRad + if(traveling) 30.deg.inRad else 0.0)

        motor.power = controller.calculate(motor.state)

    }

    //region USEFUL COMMANDS

    val spinToGreen = InstantCommand { spinTo(SpindexerSlotStatus.GREEN) }
    val spinToPurple = InstantCommand { spinTo(SpindexerSlotStatus.PURPLE) }
    val spinToIntake = InstantCommand { spinTo(SpindexerSlotStatus.EMPTY) }

    var clockwise = false


    val zeroClockwise: Command get() = InstantCommand {
        controllerDisabled = true
        motor.power = 0.1
    }

    val zeroCounterClockwise: Command get() = InstantCommand {
        controllerDisabled = true
        motor.power = -0.1
    }

    val endZero: Command get() = InstantCommand {
        motor.power = 0.0
        motor.encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
//        motor.currentPosition = 0.0
        controllerDisabled = false
        currentStatus = SpindexerStatus.TOP_0
    }

    val updateAngle: Command get() {
        return WaitUntil { controller.isWithinTolerance(KineticState(10.deg.inRad, 2.deg.inRad, Double.POSITIVE_INFINITY)) }
    }

    @Deprecated("Replace with SpinTo")
    val advanceToPurple : Command get() {
        // TODO Make slot choice smarter
        var selectedIndex = -1
        var currentIndex = 0
        slots.forEach {
            if (it == SpindexerSlotStatus.PURPLE) {
                selectedIndex = currentIndex
            }

            currentIndex++;
        }

        return if (selectedIndex == -1) {
            NullCommand()
        } else {
            SequentialGroup(
                InstantCommand { traveling = false },
                InstantCommand {
                    currentStatus = when (selectedIndex) {
                        0 -> SpindexerStatus.TOP_0
                        1 -> SpindexerStatus.TOP_1
                        2 -> SpindexerStatus.TOP_2
                        else -> currentStatus
                    }
                },
                Delay(0.1),
                updateAngle,
            )
        }
    }

    @Deprecated("Replace with SpinTo")
    val advanceToGreen : Command get() {
        // TODO Make slot choice smarter
        var selectedIndex = -1
        var currentIndex = 0
        slots.forEach {
            if (it == SpindexerSlotStatus.GREEN) {
                selectedIndex = currentIndex
            }

            currentIndex++;
        }

        return if (selectedIndex == -1) {
            NullCommand()
        } else {
            SequentialGroup(
                InstantCommand { traveling = false },
                InstantCommand { currentStatus = when (selectedIndex) {
                    0 -> SpindexerStatus.TOP_0
                    1 -> SpindexerStatus.TOP_1
                    2 -> SpindexerStatus.TOP_2
                    else -> currentStatus
                } },
                updateAngle
            )
        }
    }

    @Deprecated("Replace with SpinTo")
    val advanceToIntake : Command get() {
        // TODO Make slot choice smarter
        var selectedIndex = -1
        var currentIndex = 0
        slots.forEach {
            if (it == SpindexerSlotStatus.EMPTY) {
                selectedIndex = currentIndex
            }

            currentIndex++;
        }

        return if (selectedIndex == -1) {
            NullCommand()
        } else {
            SequentialGroup(
                InstantCommand { traveling = false },
                InstantCommand { currentStatus = when (selectedIndex) {
                    0 -> SpindexerStatus.BOTTOM_0
                    1 -> SpindexerStatus.BOTTOM_1
                    2 -> SpindexerStatus.BOTTOM_2
                    else -> currentStatus
                } },
                updateAngle
            )
        }
    }

    val enableTraveling : Command = InstantCommand { traveling = true }

    fun ticksToAngle(ticks: Double): Angle {
        return ((2 * PI * ticks) / ticksPerRev).rad
    }

    lateinit var newTarget: SpindexerStatus
    fun spinTo(goal: SpindexerSlotStatus) {
        // Decide which slot to go to
        var selection = -1
        var current = 0
        slots.forEach {
            if (it == goal) {
                selection = current
            } else {
                current++
            }
        }

        newTarget = if (goal == SpindexerSlotStatus.EMPTY) {
            when (selection) {
                0 -> SpindexerStatus.BOTTOM_0
                1 -> SpindexerStatus.BOTTOM_1
                2 -> SpindexerStatus.BOTTOM_2
                else -> currentStatus
            }
        } else {
            when (selection) {
                0 -> SpindexerStatus.TOP_0
                1 -> SpindexerStatus.TOP_1
                2 -> SpindexerStatus.TOP_2
                else -> currentStatus
            }
        }

        traveling = false
        // Set goal to that slot
        currentStatus = newTarget
    }

    class SpinTo(val goal: SpindexerSlotStatus) : Command() {
        override val isDone: Boolean
            get() = controller.isWithinTolerance(KineticState(10.deg.inRad, 2.deg.inRad, Double.POSITIVE_INFINITY))

        lateinit var target: SpindexerStatus

        override fun start() {
            // Decide which slot to go to
            var selection = -1
            var current = 0
            slots.forEach {
                if (it == goal) {
                    selection = current
                } else {
                    current++
                }
            }

            target = if (goal == SpindexerSlotStatus.EMPTY) {
                when (selection) {
                    0 -> SpindexerStatus.BOTTOM_0
                    1 -> SpindexerStatus.BOTTOM_1
                    2 -> SpindexerStatus.BOTTOM_2
                    else -> currentStatus
                }
            } else {
                when (selection) {
                    0 -> SpindexerStatus.TOP_0
                    1 -> SpindexerStatus.TOP_1
                    2 -> SpindexerStatus.TOP_2
                    else -> currentStatus
                }
            }

            // Set goal to that slot
            currentStatus = target
        }
    }
}
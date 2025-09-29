package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.math.MathFunctions.normalizeAngle
import com.qualcomm.hardware.lynx.commands.core.LynxSetPWMEnableCommand
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.AngleType
import dev.nextftc.control.feedback.AngularFeedback
import dev.nextftc.control.feedback.FeedbackType
import dev.nextftc.control.feedback.PIDElement
import dev.nextftc.control.filters.Filter
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.conditionals.switchCommand
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.NullCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.RunToPosition
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.DecoupledMotorEx
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.roundToInt

@Configurable
object Spindexer : Subsystem {

    enum class SpindexerSlotStatus {
        EMPTY,
        PURPLE,
        GREEN
    }

    enum class CurrentSpindexerStatus(
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

    var currentStatus = CurrentSpindexerStatus.TOP_0
    var traveling = false

    val ticksPerRev = 1425.1

    val motor = DecoupledMotorEx("motor_e0", "motor_c1")

    override fun initialize() {
        motor.encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        controller.goal = KineticState(0.0)
        currentStatus = CurrentSpindexerStatus.TOP_0
        slots = arrayOf(SpindexerSlotStatus.GREEN, SpindexerSlotStatus.PURPLE, SpindexerSlotStatus.PURPLE)
//        motor.zero()
    }

    @JvmField
    val controller = controlSystem {
        posFilter {
            it.custom { ticks ->
                ticksToAngle(ticks).inRad
            }
        }

        feedback(
            AngularFeedback(AngleType.RADIANS,
                PIDElement(FeedbackType.POSITION,
                    0.35, 0.0, 0.0
                )
            )
        )
    }

    override fun periodic() {
        motor.power = controller.calculate(motor.state)

        ActiveOpMode.telemetry.addData("Raw Encoder", motor.state.position)
        ActiveOpMode.telemetry.addData("Angle", ticksToAngle(motor.state.position).inDeg)
        ActiveOpMode.telemetry.addData("Spindexer goal", controller.goal.position.rad.inDeg)
        ActiveOpMode.telemetry.update()
    }

    // Given an input angle, go to that angle
    fun angleToTicks(angle: Angle): Double {
        // Assume 0 ticks is 0rad
        return (angle.inRad * ticksPerRev) / (2 * PI)
    }

    fun ticksToAngle(ticks: Double): Angle {
        return ((2 * PI * ticks) / ticksPerRev).rad
    }

    fun setAngle(angle: Angle): Command {
        return RunToPosition(controller, angle.inRad)
    }

    val updateAngle: Command get() {
        return RunToPosition(controller, (currentStatus.angle - if (traveling) 30.deg else 0.deg).inRad)
    }

    val advanceToPurple : Command get() {
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
            ParallelGroup(
                updateAngle,
                InstantCommand { currentStatus = when (selectedIndex) {
                    0 -> CurrentSpindexerStatus.TOP_0
                    1 -> CurrentSpindexerStatus.TOP_1
                    2 -> CurrentSpindexerStatus.TOP_2
                    else -> currentStatus
                } }
            )
        }
    }

    val advanceToGreen : Command get() {
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
            ParallelGroup(
                updateAngle,
                InstantCommand { currentStatus = when (selectedIndex) {
                    0 -> CurrentSpindexerStatus.TOP_0
                    1 -> CurrentSpindexerStatus.TOP_1
                    2 -> CurrentSpindexerStatus.TOP_2
                    else -> currentStatus
                } }
            )
        }
    }

    val advanceToIntake : Command get() {
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
            ParallelGroup(
                updateAngle,
                InstantCommand { currentStatus = when (selectedIndex) {
                    0 -> CurrentSpindexerStatus.BOTTOM_0
                    1 -> CurrentSpindexerStatus.BOTTOM_1
                    2 -> CurrentSpindexerStatus.BOTTOM_2
                    else -> currentStatus
                } }
            )
        }
    }

    val enableTravel = SequentialGroup(
        InstantCommand { traveling = true },
        updateAngle
    )

    val disableTravel = SequentialGroup(
        InstantCommand { traveling = false },
        updateAngle
    )

    /*
        val advanceToGreen : Command get() {
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
                setAngle((selectedIndex * 120).deg)
            }
        }

        val advanceToPurple : Command get() {
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
                setAngle((selectedIndex * 120).deg)
            }
        }

        val advanceToIntake : Command get() {
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
                setAngle(((selectedIndex * 120)+180).deg)
            }
        }

        val advanceToTravelPosition : Command get() {
            // Find which angle we're closest to (0, 120, or -120 degrees)
            val currentAngle = ticksToAngleNormalized(motor.currentPosition)
            val distToZero = abs(0 - currentAngle)
            val distTo2Pi3 = abs(((2 * PI) / 3) - currentAngle)
            val distToNegative2Pi3 = abs((-(2 * PI) / 3) - currentAngle)

            return setAngle(if (distToZero < distTo2Pi3) {
                if (distToZero < distToNegative2Pi3) {
                    PI / 6
                } else {
                    (-5 * PI) / 6
                }
            } else {
                PI / 2
            }.rad)
        }

        fun setAngle(angle: Angle) : Command {
            return RunToPosition(controller, calculateTargetTicks(angle.normalized.inRad))
        }


        //region Angle Calculation Stuff
        private fun ticksToAngle(ticks: Double): Double {
            return (2 * PI * ticks) / ticksPerRev
        }
        private fun angleToTicks(angle: Double): Double {
            return ((angle * ticksPerRev) / (2 * PI))
        }
        private fun ticksToAngleNormalized(ticks: Double): Double {
            var normalized = ticksToAngle(ticks) % (2 * PI)
            if (normalized < 0) {
                normalized += 2 * PI
            }
            return normalized
        }
        private fun calculateTargetTicks(desiredAngle: Double): Double {
            val difference = desiredAngle - ticksToAngleNormalized(motor.currentPosition)
            return if (difference > 0) {
                // COUNTERCLOCKWISE (probably)
                angleToTicks(ticksToAngle(motor.currentPosition) - difference)
            } else {
                // CLOCKWISE
                angleToTicks(ticksToAngle(motor.currentPosition) + difference)
            }
        }
        //endregion

     */
}
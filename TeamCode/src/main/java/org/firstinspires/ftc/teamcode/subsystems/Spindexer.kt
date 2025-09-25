package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.math.MathFunctions.normalizeAngle
import com.qualcomm.hardware.lynx.commands.core.LynxSetPWMEnableCommand
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.NullCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.RunToPosition
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.roundToInt

@Configurable
object Spindexer : Subsystem {

    enum class SpindexerSlotStatus {
        EMPTY,
        PURPLE,
        GREEN
    }

    var slots : Array<SpindexerSlotStatus> = arrayOf(SpindexerSlotStatus.GREEN, SpindexerSlotStatus.PURPLE, SpindexerSlotStatus.PURPLE)

    val ticksPerRev = 1425.1

    val motor = MotorEx("motor_e0").zeroed()
    val encoder = MotorEx("motor_c1").zeroed()

    @JvmField
    val controller = controlSystem {
        posPid(0.001, 0.0, 0.0)
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
        val currentAngle = ticksToAngleNormalized(encoder.currentPosition)
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

    override fun periodic() {
        val raw = controller.calculate(encoder.state)
        motor.power = if (raw > 0.25) 0.25 else raw

        ActiveOpMode.telemetry.addData("Raw Encoder", encoder.state.position)
        ActiveOpMode.telemetry.addData("Angle", raw)
        ActiveOpMode.telemetry.update()
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
        val difference = desiredAngle - ticksToAngleNormalized(encoder.currentPosition)
        return if (difference > 0) {
            // COUNTERCLOCKWISE (probably)
            angleToTicks(ticksToAngle(encoder.currentPosition) - difference)
        } else {
            // CLOCKWISE
            angleToTicks(ticksToAngle(encoder.currentPosition) + difference)
        }
    }
    //endregion
}
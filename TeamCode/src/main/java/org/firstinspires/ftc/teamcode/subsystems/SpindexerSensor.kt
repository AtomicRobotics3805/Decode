package org.firstinspires.ftc.teamcode.subsystems

import android.graphics.Color
import com.qualcomm.hardware.rev.RevColorSensorV3
import dev.nextftc.control.KineticState
import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.deg
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

object SpindexerSensor : Subsystem {

    lateinit var sensor: RevColorSensorV3

    var distanceThreshold = 3 // CM

    private var hsv = FloatArray(3)

    public var pause = false

    override fun initialize() {
        sensor = ActiveOpMode.hardwareMap.get(RevColorSensorV3::class.java, "spindexer_sensor")
        sensor.enableLed(true)
    }

    override fun periodic() {
//        currentLoop++
//
//        if (!pause) {
////        if (currentLoop == FREQUENCY) {
////            currentLoop = 0
//
//            if (!Spindexer.currentStatus.onTop && Spindexer.controller.isWithinTolerance(
//                    KineticState(5.deg.inRad)
//                )) {
//                Spindexer.slots[Spindexer.currentStatus.id] =
//                    if (sensor.getDistance(DistanceUnit.CM) <= distanceThreshold) {
//                        Color.RGBToHSV(sensor.red(), sensor.green(), sensor.blue(), hsv)
//
//                         if (hsv[0] <= 160) Spindexer.SpindexerSlotStatus.GREEN
//                         else Spindexer.SpindexerSlotStatus.PURPLE
//                    } else {
//                       Spindexer.SpindexerSlotStatus.EMPTY
//                    }
//            }
////        }
//
//        }

        ActiveOpMode.telemetry.addData("Current slot color", Spindexer.slots[Spindexer.currentStatus.id])
        ActiveOpMode.telemetry.addData("Raw distance sensor", sensor.getDistance(DistanceUnit.CM))
        ActiveOpMode.telemetry.addData("Raw hsv hue", hsv[0])
        ActiveOpMode.telemetry.addData("Spindexer in tolerance", Spindexer.controller.isWithinTolerance(
            KineticState(20.deg.inRad, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY)
        ))
        ActiveOpMode.telemetry.addData("Pause", pause)
    }

    class Read: Command() {
        override val isDone: Boolean get() {

            if (Spindexer.slots[Spindexer.currentStatus.id] == last && last != Spindexer.SpindexerSlotStatus.EMPTY) {
                numTrue++
            } else {
                last = Spindexer.slots[Spindexer.currentStatus.id]
                numTrue = 0
            }

            return numTrue > 3
        }

        var last = Spindexer.SpindexerSlotStatus.EMPTY
        var numTrue = 0

        override fun update() {
            // TODO: Only check every X loops (probably 3)

            Spindexer.slots[Spindexer.currentStatus.id] =
                if (sensor.getDistance(DistanceUnit.CM) <= distanceThreshold) {
                    Color.RGBToHSV(sensor.red(), sensor.green(), sensor.blue(), hsv)

                    if (hsv[0] <= 160) Spindexer.SpindexerSlotStatus.GREEN
                    else Spindexer.SpindexerSlotStatus.PURPLE
                } else {
                    Spindexer.SpindexerSlotStatus.EMPTY
                }
        }
    }
}
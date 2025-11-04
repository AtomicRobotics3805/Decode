package org.firstinspires.ftc.teamcode.autos

import dev.nextftc.core.units.deg
import org.firstinspires.ftc.teamcode.subsystems.Spindexer

object AutonomousInfo {
    var redAuto = false

    var finalHeading = 0.0.deg.inRad

    var autonomousRun = false

    var finalSpindexerStatus = arrayOf(Spindexer.SpindexerSlotStatus.GREEN, Spindexer.SpindexerSlotStatus.PURPLE, Spindexer.SpindexerSlotStatus.PURPLE)
}
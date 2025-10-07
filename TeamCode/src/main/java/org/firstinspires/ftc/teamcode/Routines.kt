package org.firstinspires.ftc.teamcode

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.conditionals.IfElseCommand
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.ForcedParallelCommand
import dev.nextftc.core.commands.utility.InstantCommand
import org.firstinspires.ftc.teamcode.Routines.intake
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.PusherArm
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSensor

object Routines {
    val shoot
        get() = SequentialGroup(
            Shooter.start.withDeadline(Delay(2.0)),
            PusherArm.push,
            Shooter.stop,
//            InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY }
        )

    val intake: Command
        get() = SequentialGroup(
            ParallelGroup(
                Spindexer.advanceToIntake,
                Intake.start
            ),
            SpindexerSensor.Read(),
            haltIntake
        )

    val haltIntake = ParallelGroup(
        // Spindexer.toTravel
        Intake.stop
    )

}
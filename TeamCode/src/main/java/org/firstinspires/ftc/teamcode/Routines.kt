package org.firstinspires.ftc.teamcode

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.conditionals.IfElseCommand
import dev.nextftc.core.commands.conditionals.SwitchCommand
import dev.nextftc.core.commands.conditionals.switchCommand
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.ParallelRaceGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.ForcedParallelCommand
import dev.nextftc.core.commands.utility.InstantCommand
import org.firstinspires.ftc.teamcode.Routines.intake
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.LimeLight
import org.firstinspires.ftc.teamcode.subsystems.PusherArm
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSensor
import org.firstinspires.ftc.teamcode.subsystems.LimeLight.Motif

object Routines {
    val shoot
        get() = SequentialGroup(
            Shooter.start.withDeadline(WaitUntil { Shooter.motor.velocity > Shooter.controller.goal.velocity }),
            InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY },
            PusherArm.push,
            Shooter.stop
//            InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY }
        )

    val intake: Command
        get() = SequentialGroup(
            ParallelGroup(
                Spindexer.advanceToIntake,
                Intake.start
            ),
            ParallelRaceGroup(
                SpindexerSensor.Read(),
//                Spindexer.wiggleThing
            ),
//            haltIntake
            Intake.stop
        )

    val haltIntake = ParallelGroup(
        // Spindexer.toTravel
        Intake.stop
    )

    val motifShoot: Command
        get() = switchCommand({ LimeLight.matchMotif }) {
            case(Motif.GPP, GPPMotifShoot)
            case(Motif.PGP, PGPMotifShoot)
            case(Motif.PPG, PPGMotifShoot)
        }

    val GPPMotifShoot: Command
        get() = SequentialGroup(
            Spindexer.advanceToGreen,
            Shooter.start.withDeadline(Delay(2.0)),
            PusherArm.push,
            InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY },
            Spindexer.advanceToPurple,
            Shooter.start.withDeadline(Delay(2.0)),
            PusherArm.push,
            InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY },
            Spindexer.advanceToPurple,
            Shooter.start.withDeadline(Delay(2.0)),
            PusherArm.push,
            InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY },
            Spindexer.advanceToTravel
        )

    val PGPMotifShoot: Command
        get() = SequentialGroup(
            Spindexer.advanceToPurple,
            Shooter.start.withDeadline(Delay(2.0)),
            PusherArm.push,
            InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY },
            Spindexer.advanceToGreen,
            Shooter.start.withDeadline(Delay(2.0)),
            PusherArm.push,
            InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY },
            Spindexer.advanceToPurple,
            Shooter.start.withDeadline(Delay(2.0)),
            PusherArm.push,
            InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY },
            Spindexer.advanceToTravel
        )

    val PPGMotifShoot: Command
        get() = SequentialGroup(
            Spindexer.advanceToPurple,
            Shooter.start.withDeadline(Delay(2.0)),
            PusherArm.push,
            InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY },
            Spindexer.advanceToPurple,
            Shooter.start.withDeadline(Delay(2.0)),
            PusherArm.push,
            InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY },
            Spindexer.advanceToGreen,
            Shooter.start.withDeadline(Delay(2.0)),
            PusherArm.push,
            InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY },
            Spindexer.advanceToTravel
        )

}
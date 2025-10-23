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
import dev.nextftc.core.commands.utility.LambdaCommand
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
                Spindexer.spinToIntake,
                Intake.start
            ),
            ParallelRaceGroup(
                SpindexerSensor.Read()
            ),
//            haltIntake
            Intake.stop
        )

    val haltIntake = ParallelGroup(
        // Spindexer.toTravel
        Intake.stop
    )

    val shootPurpleNoStop: Command
        get() = SequentialGroup(
            Spindexer.spinToPurple,
            Delay(1.0),
            Shooter.start.withDeadline(WaitUntil { Shooter.motor.velocity > Shooter.controller.goal.velocity }),
            InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY },
            PusherArm.push,
            Delay(1.0)
        )

    val shootGreenNoStop: Command
        get() = SequentialGroup(
            Spindexer.spinToGreen,
            Delay(1.0),
            Shooter.start.withDeadline(WaitUntil { Shooter.motor.velocity > Shooter.controller.goal.velocity }),
            InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY },
            PusherArm.push,
            Delay(1.0)
        )

    var selected = GPPMotifShoot
    val motifShoot: Command
        get() = InstantCommand {
            selected = when (LimeLight.matchMotif) {
                Motif.GPP -> GPPMotifShoot
                Motif.PGP -> PGPMotifShoot
                Motif.PPG -> PPGMotifShoot
                Motif.UNKNOWN -> selected
            }
        }.then(selected)

    val GPPMotifShoot: Command
        get() = SequentialGroup(
            shootGreenNoStop,
            shootPurpleNoStop,
            shootPurpleNoStop,
            Shooter.stop
        )

    val PGPMotifShoot: Command
        get() = SequentialGroup(
            shootPurpleNoStop,
            shootGreenNoStop,
            shootPurpleNoStop,
            Shooter.stop
        )

    val PPGMotifShoot: Command
        get() = SequentialGroup(
            shootPurpleNoStop,
            shootPurpleNoStop,
            shootGreenNoStop,
            Shooter.stop
        )
}
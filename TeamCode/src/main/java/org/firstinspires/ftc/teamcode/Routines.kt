package org.firstinspires.ftc.teamcode

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.ParallelRaceGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
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
            Intake.slowOut
        )

    val haltIntake = ParallelGroup(
        // Spindexer.toTravel
        Intake.slowOut
    )

    val shootPurpleNoStop: Command
        get() = SequentialGroup(
            ParallelGroup(
                Spindexer.spinToPurple,
                Shooter.start
            ),
            Delay(0.1),
//            WaitUntil { Shooter.motor.velocity > Shooter.controller.goal.velocity },
            InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY },
            Delay(0.5),
            PusherArm.push
        )

    val shootGreenNoStop: Command
        get() = SequentialGroup(
            ParallelGroup(
                Spindexer.spinToGreen,
                Shooter.start
            ),
            Delay(0.1),
//            WaitUntil { Shooter.motor.velocity > Shooter.controller.goal.velocity },
            InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY },
            Delay(0.5),
            PusherArm.push
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
        }.then(shootSelected)

    var selectedFinished = false

    val shootSelected: Command = LambdaCommand().setIsDone { selectedFinished }.setStart { selectedFinished = false; selected.schedule() }

    val GPPMotifShoot: Command
        get() = SequentialGroup(
            shootGreenNoStop,
            Delay(0.5),
            shootPurpleNoStop,
            Delay(0.5),
            shootPurpleNoStop,
            Shooter.stop,
            InstantCommand { selectedFinished = true }
        )

    val PGPMotifShoot: Command
        get() = SequentialGroup(
            shootPurpleNoStop,
            Delay(0.5),
            shootGreenNoStop,
            Delay(0.5),
            shootPurpleNoStop,
            Shooter.stop,
            InstantCommand { selectedFinished = true }
        )

    val PPGMotifShoot: Command
        get() = SequentialGroup(
            shootPurpleNoStop,
            Delay(0.5),
            shootPurpleNoStop,
            Delay(0.5),
            shootGreenNoStop,
            Delay(0.5),
            Shooter.stop,
            InstantCommand { selectedFinished = true }
        )
}
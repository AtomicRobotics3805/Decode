package org.firstinspires.ftc.teamcode

import dev.nextftc.control.KineticState
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.ParallelRaceGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.units.deg
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.LimeLight
import org.firstinspires.ftc.teamcode.subsystems.PusherArm
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSensor
import org.firstinspires.ftc.teamcode.subsystems.LimeLight.Motif
import org.firstinspires.ftc.teamcode.subsystems.Spindexer.SpindexerSlotStatus
import org.firstinspires.ftc.teamcode.subsystems.Spindexer.slots

object Routines {
    val shoot
        get() = SequentialGroup(
            Shooter.start.withDeadline(WaitUntil { Shooter.motor.velocity > Shooter.controller.goal.velocity }),
            Spindexer.emptyTopSlot,
            PusherArm.push,
            Shooter.stop
//            InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY }
        )

    val stopShoot
        get() = ParallelGroup(
            Shooter.stop,
            PusherArm.down
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

    /*
    val shootPurpleNoStop: Command
        get() = SequentialGroup(
            ParallelGroup(
                LambdaCommand("").setIsDone { Spindexer.controller.isWithinTolerance(KineticState(10.deg.inRad, 2.deg.inRad, Double.POSITIVE_INFINITY)) }.setStart {
                    var selectedIndex = -1
                    var currentIndex = 0
                    slots.forEach {
                        if (it == SpindexerSlotStatus.PURPLE) {
                            selectedIndex = currentIndex
                        }
                        currentIndex++;
                    }
                    if (selectedIndex != -1) {
                        Spindexer.spinToPurple()
                    } else {
                        slots.forEach {
                            if (it == SpindexerSlotStatus.GREEN) {
                                selectedIndex = currentIndex
                            }
                            currentIndex++;
                        }

                        if (selectedIndex != -1) {
                            Spindexer.spinToGreen()
                        }
                    }
                },
                Shooter.start
            ),
            Delay(0.1),
//            WaitUntil { Shooter.motor.velocity > Shooter.controller.goal.velocity },
            LambdaCommand().setIsDone { true }.setStart {
                if (Spindexer.slots[Spindexer.currentStatus.id] != Spindexer.SpindexerSlotStatus.EMPTY) {
                    Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY
                }
            },
            Delay(0.5),
            PusherArm.push
        )

    val shootGreenNoStop: Command
        get() = SequentialGroup(
            ParallelGroup(
                LambdaCommand("").setIsDone { Spindexer.controller.isWithinTolerance(KineticState(10.deg.inRad, 2.deg.inRad, Double.POSITIVE_INFINITY)) }.setStart {
                    var selectedIndex = -1
                    var currentIndex = 0
                    slots.forEach {
                        if (it == SpindexerSlotStatus.GREEN) {
                            selectedIndex = currentIndex
                        }
                        currentIndex++;
                    }
                    if (selectedIndex != -1) {
                        Spindexer.spinToGreen()
                    } else {
                        slots.forEach {
                            if (it == SpindexerSlotStatus.PURPLE) {
                                selectedIndex = currentIndex
                            }
                            currentIndex++;
                        }

                        if (selectedIndex != -1) {
                            Spindexer.spinToPurple()
                        }
                    }
                },
                Shooter.start
            ),
            Delay(0.1),
//            WaitUntil { Shooter.motor.velocity > Shooter.controller.goal.velocity },
            LambdaCommand().setIsDone { true }.setStart {
                if (Spindexer.slots[Spindexer.currentStatus.id] != Spindexer.SpindexerSlotStatus.EMPTY) {
                        Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.EMPTY
                    }
            },
            Delay(0.5),
            PusherArm.push
        )

     */

    val shootGreenNoStop get() = SequentialGroup(
        ParallelGroup(
            Spindexer.spinToGreen,
            Shooter.start
        ),
        Delay(0.2),
        ParallelGroup(
            Spindexer.emptyTopSlot,
            PusherArm.push
        )
    )

    val shootPurpleNoStop get() = SequentialGroup(
        ParallelGroup(
            Spindexer.spinToPurple,
            Shooter.start
        ),
        Delay(0.2),
        ParallelGroup(
            Spindexer.emptyTopSlot,
            PusherArm.push
        )
    )

    fun setMotifSelection() {
        selection = when (LimeLight.matchMotif) {
            Motif.GPP -> GPPMotifShoot
            Motif.PGP -> PGPMotifShoot
            Motif.PPG -> PPGMotifShoot
            Motif.UNKNOWN -> selection
        }
    }
    var selection = GPPMotifShoot
    var motifShootCompleted = false

    val motifShoot = LambdaCommand().setStart { motifShootCompleted = false; selection() }.setIsDone { motifShootCompleted }

    val GPPMotifShoot: Command
        get() = SequentialGroup(
            InstantCommand { motifShootCompleted = false },
            shootGreenNoStop,
            Delay(0.1),
            shootPurpleNoStop,
            Delay(0.1),
            shootPurpleNoStop,
            Shooter.stop,
            InstantCommand { motifShootCompleted = true }
        )

    val PGPMotifShoot: Command
        get() = SequentialGroup(
            InstantCommand { motifShootCompleted = false },
            shootPurpleNoStop,
            Delay(0.1),
            shootGreenNoStop,
            Delay(0.1),
            shootPurpleNoStop,
            Shooter.stop,
            InstantCommand { motifShootCompleted = true }
        )

    val PPGMotifShoot: Command
        get() = SequentialGroup(
            InstantCommand { motifShootCompleted = false },
            shootPurpleNoStop,
            Delay(0.1),
            shootPurpleNoStop,
            Delay(0.1),
            shootGreenNoStop,
            Shooter.stop,
            InstantCommand { motifShootCompleted = true }
        )
}
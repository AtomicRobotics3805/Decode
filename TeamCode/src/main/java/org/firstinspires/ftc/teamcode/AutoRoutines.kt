package org.firstinspires.ftc.teamcode

import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.extensions.pedro.FollowPath
import org.firstinspires.ftc.teamcode.autos.AutonomousInfo
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.LimeLight
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer

object AutoRoutines {

    val nineArtifactGoalStartAutoRoutine
        get() = SequentialGroupLocal(
            InstantCommand { AutonomousInfo.autoRunning = true },
            //region First motif
//            Spindexer.enableTraveling,
            ParallelGroup(
                FollowPath(TrajectoryFactory.goalStartToObelisk, true),
//                Shooter.stop
            ),
//            LambdaCommand().setIsDone { false }.setUpdate { ActiveOpMode.telemetry.addLine("FINISHED PATH") },
            LimeLight.detectMotif,
            ParallelGroup(
                FollowPath(TrajectoryFactory.obeliskToScore, true),
                InstantCommand(Spindexer::spinToFirstBallOfMotif),
                SequentialGroupLocal(
                    Delay(0.5),
                    Routines.motifShoot
                )
            ),
            //endregion

            //region Second motif

            // Drive to pre-pickup pos
            ParallelGroup(
                FollowPath(TrajectoryFactory.scoreToSpikeMark1, true),
                Spindexer.spinToIntake,
                SequentialGroupLocal(
                    Delay(1.0),
                    Intake.start
                )
            ),

            // Pickup ballz
            SequentialGroupLocal(
                ParallelGroup(
                    FollowPath(TrajectoryFactory.spikeMark1Pickup, true, 0.28),
                    SequentialGroupLocal(
                        Delay(0.3),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(0.7),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(1.0),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN },
                        Intake.slowOut
                    ),
                    SequentialGroupLocal(
                        Delay(1.5),
                        ParallelGroup(
                            SequentialGroupLocal(
                                Delay(0.75),
                                InstantCommand(Spindexer::spinToFirstBallOfMotif)
                            ),
                            FollowPath(TrajectoryFactory.spikeMark1PickupToScore, true)
                        )
                    )
                )
            ),

            Routines.motifShoot,


            //region Third motif

            // Drive to pre-pickup pos
            ParallelGroup(
                FollowPath(TrajectoryFactory.scoreToSpikeMark2, true),
                Spindexer.spinToIntake,
                SequentialGroupLocal(
                    Delay(1.0),
                    Intake.start
                )
            ),

            // Pickup ballz
            SequentialGroupLocal(
                ParallelGroup(
                    FollowPath(TrajectoryFactory.spikeMark2Pickup, true, 0.28),
                    SequentialGroupLocal(
                        Delay(0.3),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(0.7),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN },
                        Spindexer.spinToIntake,
                        Delay(1.0),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Intake.slowOut
                    ),
                    SequentialGroupLocal(
                        Delay(1.5),
                        ParallelGroup(
                            SequentialGroupLocal(
                                Delay(0.75),
                                InstantCommand(Spindexer::spinToFirstBallOfMotif)
                            ),
                            FollowPath(TrajectoryFactory.spikeMark2PickupToScore, true)
                        )
                    )
                )
            ),
            Routines.motifShoot,

            //endregion
            FollowPath(TrajectoryFactory.scoreToOutOfTheWay, true),
            Intake.stop,
            Shooter.actualStop
        )

    val GoalStartDumpAfterFirstAutoRoutine
        get() = SequentialGroupLocal(
            InstantCommand { AutonomousInfo.autoRunning = true },

            //region First motif
            ParallelGroup(
                FollowPath(TrajectoryFactory.goalStartToObelisk, true),
//                Shooter.stop
            ),
//            LambdaCommand().setIsDone { false }.setUpdate { ActiveOpMode.telemetry.addLine("FINISHED PATH") },
            LimeLight.detectMotif,
            ParallelGroup(
                FollowPath(TrajectoryFactory.obeliskToScore, true),
                SequentialGroupLocal(
                    Delay(0.5),
                    Routines.motifShoot
                )
            ),
            //endregion

            //region Second motif

            // Drive to pre-pickup pos
            ParallelGroup(
                FollowPath(TrajectoryFactory.scoreToSpikeMark1, true),
                Spindexer.spinToIntake,
                SequentialGroupLocal(
                    Delay(1.0),
                    Intake.start
                )
            ),

            // Pickup ballz
            SequentialGroupLocal(
                ParallelGroup(
                    FollowPath(TrajectoryFactory.spikeMark1Pickup, true, 0.25),
                    SequentialGroupLocal(
                        Delay(0.3),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(0.7),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(1.0),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN },
                        Intake.slowOut
                    ),
                    SequentialGroupLocal(
                        Delay(1.5),
                        FollowPath(TrajectoryFactory.spikeMark1ToDump, true),
                        Delay(2.0),
                        FollowPath(TrajectoryFactory.dumpToScore, true)
                    )
                )
            ),

            Routines.motifShoot,


            //region Third motif

            // Drive to pre-pickup pos
            ParallelGroup(
                FollowPath(TrajectoryFactory.scoreToSpikeMark2, true),
                Spindexer.spinToIntake,
                SequentialGroupLocal(
                    Delay(1.0),
                    Intake.start
                )
            ),

            // Pickup ballz
            SequentialGroupLocal(
                ParallelGroup(
                    FollowPath(TrajectoryFactory.spikeMark2Pickup, true, 0.25),
                    SequentialGroupLocal(
                        Delay(0.3),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(0.7),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN },
                        Spindexer.spinToIntake,
                        Delay(1.0),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Intake.slowOut
                    ),
                    SequentialGroupLocal(
                        Delay(1.5),
                        FollowPath(TrajectoryFactory.spikeMark2PickupToScore, true)
                    )
                )
            ),
            Routines.motifShoot,

            //endregion

            FollowPath(TrajectoryFactory.scoreToOutOfTheWay, true),
            Intake.stop,
            Shooter.actualStop
        )

    val GoalStartDumpAfterSecondAutoRoutine
        get() = SequentialGroupLocal(
            InstantCommand { AutonomousInfo.autoRunning = true },

            //region First motif
            ParallelGroup(
                FollowPath(TrajectoryFactory.goalStartToObelisk, true),
//                Shooter.stop
            ),
//            LambdaCommand().setIsDone { false }.setUpdate { ActiveOpMode.telemetry.addLine("FINISHED PATH") },
            LimeLight.detectMotif,
            ParallelGroup(
                FollowPath(TrajectoryFactory.obeliskToScore, true),
                SequentialGroupLocal(
                    Delay(0.5),
                    Routines.motifShoot
                )
            ),
            //endregion

            //region Second motif

            // Drive to pre-pickup pos
            ParallelGroup(
                FollowPath(TrajectoryFactory.scoreToSpikeMark1, true),
                Spindexer.spinToIntake,
                SequentialGroupLocal(
                    Delay(1.0),
                    Intake.start
                )
            ),

            // Pickup ballz
            SequentialGroupLocal(
                ParallelGroup(
                    FollowPath(TrajectoryFactory.spikeMark1Pickup, true, 0.25),
                    SequentialGroupLocal(
                        Delay(0.3),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(0.7),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(1.0),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN },
                        Intake.slowOut
                    ),
                    SequentialGroupLocal(
                        Delay(1.5),
                        FollowPath(TrajectoryFactory.spikeMark1PickupToScore)
                    )
                )
            ),

            Routines.motifShoot,


            //region Third motif

            // Drive to pre-pickup pos
            ParallelGroup(
                FollowPath(TrajectoryFactory.scoreToSpikeMark2, true),
                Spindexer.spinToIntake,
                SequentialGroupLocal(
                    Delay(1.0),
                    Intake.start
                )
            ),

            // Pickup ballz
            SequentialGroupLocal(
                ParallelGroup(
                    FollowPath(TrajectoryFactory.spikeMark2Pickup, true, 0.25),
                    SequentialGroupLocal(
                        Delay(0.3),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(0.7),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(1.0),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN },
                        Intake.slowOut
                    ),
                    SequentialGroupLocal(
                        Delay(1.5),
                        FollowPath(TrajectoryFactory.spikeMark2ToDump, true),
                        Delay(2.0),
                        FollowPath(TrajectoryFactory.dumpToScore, true)
                    )
                )
            ),
            Routines.motifShoot,

            //endregion

            FollowPath(TrajectoryFactory.scoreToOutOfTheWay, true),
            Intake.stop,
            Shooter.actualStop
        )

    val GoalStartDumpAfterBothAutoRoutine
        get() = SequentialGroupLocal(
            InstantCommand { AutonomousInfo.autoRunning = true },

            //region First motif
            ParallelGroup(
                FollowPath(TrajectoryFactory.goalStartToObelisk, true),
//                Shooter.stop
            ),
//            LambdaCommand().setIsDone { false }.setUpdate { ActiveOpMode.telemetry.addLine("FINISHED PATH") },
            LimeLight.detectMotif,
            ParallelGroup(
                FollowPath(TrajectoryFactory.obeliskToScore, true),
                SequentialGroupLocal(
                    Delay(0.5),
                    Routines.motifShoot
                )
            ),
            //endregion

            //region Second motif

            // Drive to pre-pickup pos
            ParallelGroup(
                FollowPath(TrajectoryFactory.scoreToSpikeMark1, true),
                Spindexer.spinToIntake,
                SequentialGroupLocal(
                    Delay(1.0),
                    Intake.start
                )
            ),

            // Pickup ballz
            SequentialGroupLocal(
                ParallelGroup(
                    FollowPath(TrajectoryFactory.spikeMark1Pickup, true, 0.25),
                    SequentialGroupLocal(
                        Delay(0.3),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(0.7),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(1.0),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN },
                        Intake.slowOut
                    ),
                    SequentialGroupLocal(
                        Delay(1.5),
                        ParallelGroup(
                            SequentialGroupLocal(
                                Delay(0.75),
                                InstantCommand(Spindexer::spinToFirstBallOfMotif)
                            ),
                            FollowPath(TrajectoryFactory.spikeMark1ToDump, true)
                        ),
                        FollowPath(TrajectoryFactory.dumpToScore, true)
                    )
                )
            ),

            Routines.motifShoot,


            //region Third motif

            // Drive to pre-pickup pos
            ParallelGroup(
                FollowPath(TrajectoryFactory.scoreToSpikeMark2, true),
                Spindexer.spinToIntake,
                SequentialGroupLocal(
                    Delay(1.0),
                    Intake.start
                )
            ),

            // Pickup ballz
            SequentialGroupLocal(
                ParallelGroup(
                    FollowPath(TrajectoryFactory.spikeMark2Pickup, true, 0.25),
                    SequentialGroupLocal(
                        Delay(0.3),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(0.7),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN },
                        Spindexer.spinToIntake,
                        Delay(1.0),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Intake.reverse,
                        Delay(0.5),
                        Intake.slowOut
                    ),
                    SequentialGroupLocal(
                        Delay(1.5),
                        ParallelGroup(
                            SequentialGroupLocal(
                                Delay(0.75),
                                InstantCommand(Spindexer::spinToFirstBallOfMotif)
                            ),
                            FollowPath(TrajectoryFactory.spikeMark2ToDump, true)
                        ),
                        FollowPath(TrajectoryFactory.dumpToScore, true)
                    )
                )
            ),
            Routines.motifShoot,

            //endregion

            FollowPath(TrajectoryFactory.scoreToGoalZonePark, true),
            Intake.stop,
            Shooter.actualStop
        )

    val threeArtifactFarStartAutoRoutine
        get() = SequentialGroupLocal(
            InstantCommand { AutonomousInfo.autoRunning = true },
            LimeLight.detectMotif,
            FollowPath(TrajectoryFactory.farStartToScore, true),
            Routines.motifShoot,
            FollowPath(TrajectoryFactory.farScoreToPark, true),
            Intake.stop,
            Shooter.actualStop
        )

    val sixArtifactFarStartAutoRoutine
        get() = SequentialGroupLocal(
            InstantCommand { AutonomousInfo.autoRunning = true },
            LimeLight.detectMotif,
            FollowPath(TrajectoryFactory.farStartToScore, true),
            Routines.motifShoot,
            ParallelGroup(
                FollowPath(TrajectoryFactory.spikeMark3Pickup, true, 0.25),
                SequentialGroupLocal(
                    Delay(0.3),
                    InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN },
                    Spindexer.spinToIntake,
                    Delay(0.7),
                    InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                    Spindexer.spinToIntake,
                    Delay(1.0),
                    InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                    Intake.slowOut
                ),
                SequentialGroupLocal(
                    Delay(1.25),
                    FollowPath(TrajectoryFactory.spikeMark3PickupToScore, true)
                )
            ),
            Routines.motifShoot,
            FollowPath(TrajectoryFactory.farScoreToPark, true),
            Intake.stop,
            Shooter.actualStop
        )

    val nineArtifactFarStartAutoRoutine
        get() = SequentialGroupLocal(
            InstantCommand { AutonomousInfo.autoRunning = true },
            LimeLight.detectMotif,
            FollowPath(TrajectoryFactory.farStartToScore, true),
            Routines.motifShoot,
            ParallelGroup(
                FollowPath(TrajectoryFactory.spikeMark3Pickup, true, 0.25),
                SequentialGroupLocal(
                    Delay(0.3),
                    InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN },
                    Spindexer.spinToIntake,
                    Delay(0.7),
                    InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                    Spindexer.spinToIntake,
                    Delay(1.0),
                    InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                    Intake.slowOut
                ),
                SequentialGroupLocal(
                    Delay(1.25),
                    FollowPath(TrajectoryFactory.spikeMark3PickupToScore, true)
                )
            ),
            Routines.motifShoot,
            FollowPath(TrajectoryFactory.farScoreToHumanPlayer),
            ParallelGroup(
                FollowPath(TrajectoryFactory.humanPlayerPickup, true, 0.25),
                SequentialGroupLocal(
                    Delay(0.3),
                    InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                    Spindexer.spinToIntake,
                    Delay(0.7),
                    InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN },
                    Spindexer.spinToIntake,
                    Delay(1.0),
                    InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                    Intake.slowOut
                ),
                SequentialGroupLocal(
                    Delay(1.25),
                    FollowPath(TrajectoryFactory.humanPlayerToFarScore, true)
                )
            ),
            Routines.motifShoot,
            FollowPath(TrajectoryFactory.farScoreToPark, true),
            Intake.stop,
            Shooter.actualStop
        )
}
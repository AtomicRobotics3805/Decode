package org.firstinspires.ftc.teamcode

import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.extensions.pedro.FollowPath
import org.firstinspires.ftc.teamcode.autos.AutonomousInfo
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.LimeLight
import org.firstinspires.ftc.teamcode.subsystems.Spindexer

object AutoRoutines {

    val secondVolleyDelay = 0.0
    val preFirstVolleyDelay = 0.0


    @Deprecated("Outdated")
    val sixArtifactGoalStartAutoRoutine
        get() = SequentialGroupLocal(
            //region First motif
            Spindexer.enableTraveling,
            FollowPath(TrajectoryFactoryAtInterleague.goalStartToObelisk, true),
//            LambdaCommand().setIsDone { false }.setUpdate { ActiveOpMode.telemetry.addLine("FINISHED PATH") },
            Delay(0.25),
            Delay(preFirstVolleyDelay),
            LimeLight.detectMotif,
            ParallelGroup(
                FollowPath(TrajectoryFactoryAtInterleague.obeliskToScore, true),
                SequentialGroupLocal(
                    Delay(0.5),
                    Routines.motifShoot
                )
            ),
            //endregion

            //region Second motif

            // Drive to pre-pickup pos
            ParallelGroup(
                FollowPath(TrajectoryFactoryAtInterleague.scoreToSpikeMark1, true),
                Spindexer.spinToIntake,
                Intake.start
            ),

            // Pickup first ball
            ParallelGroup(
                FollowPath(TrajectoryFactoryAtInterleague.spikeMark1Pickup1, true),
                InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE }
            ),

            // Pickup second ball
            Spindexer.spinToIntake,
            ParallelGroup(
                FollowPath(TrajectoryFactoryAtInterleague.spikeMark1Pickup2, true),
                InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE }
            ),

            // Pickup third ball
            Spindexer.spinToIntake,
            ParallelGroup(
                FollowPath(TrajectoryFactoryAtInterleague.spikeMark1Pickup3, true),
                InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN }
            ),

            // Drive to score
            ParallelGroup(
                Intake.slowOut,
                FollowPath(TrajectoryFactoryAtInterleague.spikeMark1PickupToScore, true),
                Spindexer.enableTraveling
            ),

            Delay(secondVolleyDelay),

            Routines.motifShoot,

            //endregion
            FollowPath(TrajectoryFactoryAtInterleague.scoreToOutOfTheWay, true),
            Intake.stop
        )


    val nineArtifactGoalStartAutoRoutine
        get() = SequentialGroupLocal(
            InstantCommand { AutonomousInfo.autoRunning = true },
            //region First motif
            Spindexer.enableTraveling,
            ParallelGroup(
                FollowPath(TrajectoryFactoryAtInterleague.goalStartToObelisk, true),
//                Shooter.stop
            ),
//            LambdaCommand().setIsDone { false }.setUpdate { ActiveOpMode.telemetry.addLine("FINISHED PATH") },
            LimeLight.detectMotif,
            ParallelGroup(
                FollowPath(TrajectoryFactoryAtInterleague.obeliskToScore, true),
                SequentialGroupLocal(
                    Delay(0.5),
                    Routines.motifShoot
                )
            ),
            //endregion

            //region Second motif

            // Drive to pre-pickup pos
            ParallelGroup(
                FollowPath(TrajectoryFactoryAtInterleague.scoreToSpikeMark1, true),
                Spindexer.spinToIntake,
                SequentialGroupLocal(
                    Delay(1.0),
                    Intake.start
                )
            ),

            // Pickup ballz
            SequentialGroupLocal(
                ParallelGroup(
                    FollowPath(TrajectoryFactoryAtInterleague.spikeMark1Pickup3, true),
                    SequentialGroupLocal(
                        Delay(0.5),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(1.0),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(1.0),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN },
                        Intake.slowOut
                    ),
                    SequentialGroupLocal(
                        Delay(0.6),
                        FollowPath(TrajectoryFactoryAtInterleague.spikeMark1PickupToScore, true)
                    )
                )
            ),

            Routines.motifShoot,


            //region Third motif

            // Drive to pre-pickup pos
            ParallelGroup(
                FollowPath(TrajectoryFactoryAtInterleague.scoreToSpikeMark2, true),
                Spindexer.spinToIntake,
                SequentialGroupLocal(
                    Delay(1.0),
                    Intake.start
                )
            ),

            // Pickup ballz
            SequentialGroupLocal(
                ParallelGroup(
                    FollowPath(TrajectoryFactoryAtInterleague.spikeMark2Pickup3, true),
                    SequentialGroupLocal(
                        Delay(0.5),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(1.0),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN },
                        Spindexer.spinToIntake,
                        Delay(1.0),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Intake.slowOut
                    ),
                    SequentialGroupLocal(
                        Delay(0.6),
                        FollowPath(TrajectoryFactoryAtInterleague.spikeMark2PickupToScore, true)
                    )
                )
            ),
            Routines.motifShoot,

            //endregion
            FollowPath(TrajectoryFactoryAtInterleague.scoreToOutOfTheWay, true),
            Intake.stop
        )

    val nineArtifactDumpGoalStartAutoRoutine
        get() = SequentialGroupLocal(
            InstantCommand { AutonomousInfo.autoRunning = true },
            //region First motif
            Spindexer.enableTraveling,
            ParallelGroup(
                FollowPath(TrajectoryFactoryAtInterleague.goalStartToObelisk, true),
//                Shooter.stop
            ),
//            LambdaCommand().setIsDone { false }.setUpdate { ActiveOpMode.telemetry.addLine("FINISHED PATH") },
            LimeLight.detectMotif,
            ParallelGroup(
                FollowPath(TrajectoryFactoryAtInterleague.obeliskToScore, true),
                SequentialGroupLocal(
                    Delay(0.5),
                    Routines.motifShoot
                )
            ),
            //endregion

            //region Second motif

            // Drive to pre-pickup pos
            ParallelGroup(
                FollowPath(TrajectoryFactoryAtInterleague.scoreToSpikeMark1, true),
                Spindexer.spinToIntake,
                SequentialGroupLocal(
                    Delay(1.0),
                    Intake.start
                )
            ),

            // Pickup ballz
            SequentialGroupLocal(
                ParallelGroup(
                    FollowPath(TrajectoryFactoryAtInterleague.spikeMark1Pickup3, true),
                    SequentialGroupLocal(
                        Delay(0.6),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(0.7),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(0.7),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN },
                        Intake.slowOut
                    ),
                    SequentialGroupLocal(
                        Delay(0.6),
                        FollowPath(TrajectoryFactoryAtInterleague.spikeMark1ToDump, true),
                        Delay(1.5),
                        FollowPath(TrajectoryFactoryAtInterleague.dumpToScore, true)
                    )
                )
            ),

            Routines.motifShoot,


            //region Third motif

            // Drive to pre-pickup pos
            ParallelGroup(
                FollowPath(TrajectoryFactoryAtInterleague.scoreToSpikeMark2, true),
                Spindexer.spinToIntake,
                SequentialGroupLocal(
                    Delay(1.0),
                    Intake.start
                )
            ),

            // Pickup ballz
            SequentialGroupLocal(
                ParallelGroup(
                    FollowPath(TrajectoryFactoryAtInterleague.spikeMark2Pickup3, true),
                    SequentialGroupLocal(
                        Delay(0.6),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(0.7),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN },
                        Spindexer.spinToIntake,
                        Delay(0.7),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Intake.slowOut
                    ),
                    SequentialGroupLocal(
                        Delay(0.6),
                        FollowPath(TrajectoryFactoryAtInterleague.spikeMark2PickupToScore, true)
                    )
                )
            ),
            Routines.motifShoot,

            //endregion
            FollowPath(TrajectoryFactoryAtInterleague.scoreToOutOfTheWay, true),
            Intake.stop
        )


    val farParkAutoRoutine
        get() = SequentialGroupLocal(
            InstantCommand { AutonomousInfo.autoRunning = true },
            FollowPath(TrajectoryFactoryAtInterleague.farStartToPark)
        )

    val threeArtifactFarStartAutoRoutine
        get() = SequentialGroupLocal(
            Spindexer.enableTraveling,
            ParallelGroup(
                FollowPath(TrajectoryFactoryAtInterleague.farStartToShoot, true),
                SequentialGroupLocal(
                    Delay(1.0),
                    LimeLight.detectMotif,
                    Delay(0.5),
                    Routines.motifShoot
                )
            ),
            FollowPath(TrajectoryFactoryAtInterleague.farShootPosToPark, true)
        )
}




//            PIDToPoint(TrajectoryFactory.obeliskSensePos, 6.0, 10.deg.inRad),

//            ParallelRaceGroup(WaitUntil { LimeLight.matchMotif != LimeLight.Motif.UNKNOWN}, Delay(1.0)),
//            PIDToPoint(TrajectoryFactory.scorePos, 6.0, 10.deg.inRad),
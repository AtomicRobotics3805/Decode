package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.ParallelRaceGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.units.deg
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.LimeLight
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer

object AutoRoutines {

    val threeArtifactGoalStartAutoRoutine
        get() = SequentialGroup(
            Spindexer.enableTraveling,
            FollowPath(TrajectoryFactory.goalStartToObelisk, true),
//            LambdaCommand().setIsDone { false }.setUpdate { ActiveOpMode.telemetry.addLine("FINISHED PATH") },
            Delay(0.25),
            LimeLight.detectMotif,
            ParallelGroup(
                FollowPath(TrajectoryFactory.obeliskToScore, true),
                SequentialGroup(
                    Delay(0.5),
                    Routines.motifShoot
                )
            )
        )

    val secondVolleyDelay = 0.0
    val preFirstVolleyDelay = 0.0

    val sixArtifactGoalStartAutoRoutine
        get() = SequentialGroup(
            //region First motif
            Spindexer.enableTraveling,
            FollowPath(TrajectoryFactory.goalStartToObelisk, true),
//            LambdaCommand().setIsDone { false }.setUpdate { ActiveOpMode.telemetry.addLine("FINISHED PATH") },
            Delay(0.25),
            Delay(preFirstVolleyDelay),
            LimeLight.detectMotif,
            ParallelGroup(
                FollowPath(TrajectoryFactory.obeliskToScore, true),
                SequentialGroup(
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
                Intake.start
            ),

            // Pickup first ball
            ParallelGroup(
                FollowPath(TrajectoryFactory.spikeMark1Pickup1, true),
                InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE }
            ),

            // Pickup second ball
            Spindexer.spinToIntake,
            ParallelGroup(
                FollowPath(TrajectoryFactory.spikeMark1Pickup2, true),
                InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE }
            ),

            // Pickup third ball
            Spindexer.spinToIntake,
            ParallelGroup(
                FollowPath(TrajectoryFactory.spikeMark1Pickup3, true),
                InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN }
            ),

            // Drive to score
            ParallelGroup(
                Intake.slowOut,
                FollowPath(TrajectoryFactory.spikeMark1PickupToScore, true),
                Spindexer.enableTraveling
            ),

            Delay(secondVolleyDelay),

            Routines.motifShoot,

            //endregion
            FollowPath(TrajectoryFactory.scoreToOutOfTheWay, true),
            Intake.stop
        )


    val nineArtifactGoalStartAutoRoutine
        get() = SequentialGroup(
            //region First motif
            Spindexer.enableTraveling,
            ParallelGroup(
                FollowPath(TrajectoryFactory.goalStartToObelisk, true),
//                Shooter.stop
            ),
//            LambdaCommand().setIsDone { false }.setUpdate { ActiveOpMode.telemetry.addLine("FINISHED PATH") },
            LimeLight.detectMotif,
            ParallelGroup(
                FollowPath(TrajectoryFactory.obeliskToScore, true),
                SequentialGroup(
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
                SequentialGroup(
                    Delay(1.0),
                    Intake.start
                )
            ),

            // Pickup ballz
            SequentialGroup(
                ParallelGroup(
                    FollowPath(TrajectoryFactory.spikeMark1Pickup3, true),
                    SequentialGroup(
                        Delay(0.3),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(0.7),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(0.7),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN },
                        Intake.slowOut
                    ),
                    SequentialGroup(
                        Delay(0.6),
                        FollowPath(TrajectoryFactory.spikeMark1PickupToScore, true)
                    )
                )
            ),

            Routines.motifShoot,


            //region Third motif

            // Drive to pre-pickup pos
            ParallelGroup(
                FollowPath(TrajectoryFactory.scoreToSpikeMark2, true),
                Spindexer.spinToIntake,
                SequentialGroup(
                    Delay(1.0),
                    Intake.start
                )
            ),

            // Pickup ballz
            SequentialGroup(
                ParallelGroup(
                    FollowPath(TrajectoryFactory.spikeMark2Pickup3, true),
                    SequentialGroup(
                        Delay(0.4),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(0.7),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN },
                        Spindexer.spinToIntake,
                        Delay(0.7),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Intake.slowOut
                    ),
                    SequentialGroup(
                        Delay(0.6),
                        FollowPath(TrajectoryFactory.spikeMark2PickupToScore, true)
                    )
                )
            ),
            Routines.motifShoot,

            //endregion
            FollowPath(TrajectoryFactory.scoreToOutOfTheWay, true),
            Intake.stop
        )

    val nineArtifactDumpGoalStartAutoRoutine
        get() = SequentialGroup(
            //region First motif
            Spindexer.enableTraveling,
            ParallelGroup(
                FollowPath(TrajectoryFactory.goalStartToObelisk, true),
//                Shooter.stop
            ),
//            LambdaCommand().setIsDone { false }.setUpdate { ActiveOpMode.telemetry.addLine("FINISHED PATH") },
            LimeLight.detectMotif,
            ParallelGroup(
                FollowPath(TrajectoryFactory.obeliskToScore, true),
                SequentialGroup(
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
                SequentialGroup(
                    Delay(1.0),
                    Intake.start
                )
            ),

            // Pickup ballz
            SequentialGroup(
                ParallelGroup(
                    FollowPath(TrajectoryFactory.spikeMark1Pickup3, true),
                    SequentialGroup(
                        Delay(0.3),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(0.7),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(0.7),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN },
                        Intake.slowOut
                    ),
                    SequentialGroup(
                        Delay(0.6),
                        FollowPath(TrajectoryFactory.spikeMark1ToDump, true),
                        Delay(1.5),
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
                SequentialGroup(
                    Delay(1.0),
                    Intake.start
                )
            ),

            // Pickup ballz
            SequentialGroup(
                ParallelGroup(
                    FollowPath(TrajectoryFactory.spikeMark2Pickup3, true),
                    SequentialGroup(
                        Delay(0.4),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Spindexer.spinToIntake,
                        Delay(0.7),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN },
                        Spindexer.spinToIntake,
                        Delay(0.7),
                        InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                        Intake.slowOut
                    ),
                    SequentialGroup(
                        Delay(0.6),
                        FollowPath(TrajectoryFactory.spikeMark2PickupToScore, true)
                    )
                )
            ),
            Routines.motifShoot,

            //endregion
            FollowPath(TrajectoryFactory.scoreToOutOfTheWay, true),
            Intake.stop
        )


    val farParkAutoRoutine
        get() = SequentialGroup(
            FollowPath(TrajectoryFactory.farStartToPark)
        )

    val threeArtifactFarStartAutoRoutine
        get() = SequentialGroup(
            Spindexer.enableTraveling,
            ParallelGroup(
                FollowPath(TrajectoryFactory.farStartToShoot, true),
                SequentialGroup(
                    Delay(1.0),
                    LimeLight.detectMotif,
                    Delay(0.5),
                    Routines.motifShoot
                )
            ),
            FollowPath(TrajectoryFactory.farShootPosToPark, true)
        )
}




//            PIDToPoint(TrajectoryFactory.obeliskSensePos, 6.0, 10.deg.inRad),

//            ParallelRaceGroup(WaitUntil { LimeLight.matchMotif != LimeLight.Motif.UNKNOWN}, Delay(1.0)),
//            PIDToPoint(TrajectoryFactory.scorePos, 6.0, 10.deg.inRad),
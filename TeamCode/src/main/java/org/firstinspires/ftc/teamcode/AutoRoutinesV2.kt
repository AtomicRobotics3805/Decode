package org.firstinspires.ftc.teamcode

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.extensions.pedro.FollowPath
import org.firstinspires.ftc.teamcode.autos.AutonomousInfo
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.LimeLight
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.subsystems.Spindexer.spinToFirstBallOfMotif

object AutoRoutinesV2 {
    //region Building Blocks
    val startThroughFirstPickup: Command
        get() = SequentialGroupLocal(
            InstantCommand { Callbacks.reset(); AutonomousInfo.autoRunning = true },
            FollowPath(TrajectoryFactory.goalStartToObelisk, true),

            LimeLight.detectMotif,
            ParallelGroup(
                FollowPath(TrajectoryFactory.obeliskToScore, true),
                InstantCommand(Spindexer::spinToFirstBallOfMotif),
                SequentialGroupLocal(
                    Delay(0.5),
                    Routines.motifShoot
                )
            ),

            ParallelGroup(
                FollowPath(TrajectoryFactory.scoreToSpikeMark1, true, 0.75),
                Spindexer.spinToIntake,
                SequentialGroupLocal(
                    Delay(0.25),
                    Intake.start
                )
            ),

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
                        Callbacks.setCallback(Callbacks.Callback.FIRST_PICKUP_COMPLETE)
                    )
                )
            )
        )

    val secondVolleyShootThroughSecondPickup: Command
        get() = SequentialGroupLocal(
            Routines.motifShoot,

            ParallelGroup(
                FollowPath(TrajectoryFactory.scoreToSpikeMark2, true, 0.75),
                Spindexer.spinToIntake,
                SequentialGroupLocal(
                    Delay(1.0),
                    Intake.start
                )
            ),

            ParallelGroup(
                FollowPath(TrajectoryFactory.spikeMark2Pickup, true, 0.25),
                SequentialGroupLocal(
                    Delay(0.3),
                    InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                    Spindexer.spinToIntake,
                    Delay(0.75),
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
                            Delay(0.9),
                            InstantCommand(Spindexer::spinToFirstBallOfMotif)
                        ),
                        Callbacks.setCallback(Callbacks.Callback.SECOND_PICKUP_COMPLETE)
                    )
                )
            )
        )

    val thirdVolleyShootThroughNormalPark: Command
        get() = SequentialGroupLocal(
            Routines.motifShoot,

            FollowPath(TrajectoryFactory.scoreToOutOfTheWay, true),
            Intake.stop,
            Shooter.actualStop
        )

    val thirdVolleyShootThroughGoalZonePark: Command
        get() = SequentialGroupLocal(
            Routines.motifShoot,

            FollowPath(TrajectoryFactory.scoreToGoalZonePark, true),
            Intake.stop,
            Shooter.actualStop
        )

    //endregion

    val goalStartNine: Command
        get() = SequentialGroupLocal(
            ParallelGroup(
                startThroughFirstPickup,
                SequentialGroupLocal(
                    Callbacks.waitFor(Callbacks.Callback.FIRST_PICKUP_COMPLETE),
                    FollowPath(TrajectoryFactory.spikeMark1PickupToScore, true)
                )
            ),
            ParallelGroup(
                secondVolleyShootThroughSecondPickup,
                SequentialGroupLocal(
                    Callbacks.waitFor(Callbacks.Callback.SECOND_PICKUP_COMPLETE),
                    FollowPath(TrajectoryFactory.spikeMark2PickupToScore, true)
                )
            ),
            thirdVolleyShootThroughNormalPark
        )

    val goalStartNineDumpFirst: Command
        get() = SequentialGroupLocal(
            ParallelGroup(
                startThroughFirstPickup,
                SequentialGroupLocal(
                    Callbacks.waitFor(Callbacks.Callback.FIRST_PICKUP_COMPLETE),
                    ParallelGroup(
                        Spindexer.enableTraveling,
                        FollowPath(TrajectoryFactory.spikeMark1ToDump, true),
                    ),
                    Delay(2.0),
                    ParallelGroup(
                        InstantCommand { Spindexer.spinToFirstBallOfMotif() },
                        FollowPath(TrajectoryFactory.firstDumpToScore, true)
                    )
                )
            ),
            ParallelGroup(
                secondVolleyShootThroughSecondPickup,
                SequentialGroupLocal(
                    Callbacks.waitFor(Callbacks.Callback.SECOND_PICKUP_COMPLETE),
                    FollowPath(TrajectoryFactory.spikeMark2PickupToScore, true)
                )
            ),
            thirdVolleyShootThroughNormalPark
        )

    val goalStartNineDumpSecond: Command
        get() = SequentialGroupLocal(
            ParallelGroup(
                startThroughFirstPickup,
                SequentialGroupLocal(
                    Callbacks.waitFor(Callbacks.Callback.FIRST_PICKUP_COMPLETE),
                    FollowPath(TrajectoryFactory.spikeMark1PickupToScore, true)
                )
            ),
            ParallelGroup(
                secondVolleyShootThroughSecondPickup,
                SequentialGroupLocal(
                    Callbacks.waitFor(Callbacks.Callback.SECOND_PICKUP_COMPLETE),
                    FollowPath(TrajectoryFactory.spikeMark2ToDump, true),
                    Delay(2.0),
                    FollowPath(TrajectoryFactory.secondDumpToScore, true)
                )
            ),
            thirdVolleyShootThroughNormalPark
        )

    val goalStartNineDumpBoth: Command
        get() = SequentialGroupLocal(
            ParallelGroup(
                startThroughFirstPickup,
                SequentialGroupLocal(
                    Callbacks.waitFor(Callbacks.Callback.FIRST_PICKUP_COMPLETE),
                    FollowPath(TrajectoryFactory.spikeMark1ToDump, true),
                    Delay(1.0),
                    FollowPath(TrajectoryFactory.firstDumpToScore, true)
                )
            ),
            ParallelGroup(
                secondVolleyShootThroughSecondPickup,
                SequentialGroupLocal(
                    Callbacks.waitFor(Callbacks.Callback.SECOND_PICKUP_COMPLETE),
                    FollowPath(TrajectoryFactory.spikeMark2ToDump, true),
                    Delay(1.0),
                    FollowPath(TrajectoryFactory.secondDumpToScore, true)
                )
            ),
            thirdVolleyShootThroughNormalPark
        )
}
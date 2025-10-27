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

    val sixArtifactGoalStartAutoRoutine
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
            ),

            // GO PICKUP BALLS
            ParallelGroup(
                FollowPath(TrajectoryFactory.scoreToSpikeMark1, true),
                Spindexer.spinToIntake,
                Intake.start
            ),

            SequentialGroup(
                FollowPath(TrajectoryFactory.spikeMark1Pickup1, true),
                InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                ParallelGroup(
                    Spindexer.spinToIntake,
                    FollowPath(TrajectoryFactory.spikeMark1Pickup2, true)
                ),
                InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.PURPLE },
                ParallelGroup(
                    Spindexer.spinToIntake,
                    FollowPath(TrajectoryFactory.spikeMark1Pickup3, true)
                ),
                InstantCommand { Spindexer.slots[Spindexer.currentStatus.id] = Spindexer.SpindexerSlotStatus.GREEN },
            ),
            ParallelGroup(
                Intake.slowOut,
                FollowPath(TrajectoryFactory.spikeMark1PickupToScore, true),
                Spindexer.enableTraveling
            ),
            Routines.motifShoot
        )

    val justThePaths
        get() = SequentialGroup(
            FollowPath(TrajectoryFactory.goalStartToObelisk, true),
            FollowPath(TrajectoryFactory.obeliskToScore, true),
            Routines.GPPMotifShoot,
            FollowPath(TrajectoryFactory.scoreToSpikeMark1, true)
        )
}




//            PIDToPoint(TrajectoryFactory.obeliskSensePos, 6.0, 10.deg.inRad),

//            ParallelRaceGroup(WaitUntil { LimeLight.matchMotif != LimeLight.Motif.UNKNOWN}, Delay(1.0)),
//            PIDToPoint(TrajectoryFactory.scorePos, 6.0, 10.deg.inRad),
package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.units.deg
import dev.nextftc.extensions.pedro.FollowPath
import org.firstinspires.ftc.teamcode.subsystems.LimeLight
import org.firstinspires.ftc.teamcode.subsystems.Spindexer

object AutoRoutines {

    val threeArtifactGoalStartAutoRoutine get() = SequentialGroup(
        PIDToPoint(TrajectoryFactory.obeliskSensePos, 6.0, 10.deg.inRad),
//        FollowPath(TrajectoryFactory.goalStartToObelisk, true),
        LimeLight.detectMotif,
        PIDToPoint(TrajectoryFactory.scorePos, 6.0, 10.deg.inRad),
        Routines.motifShoot
//        Spindexer.spinToPurple
    )
}
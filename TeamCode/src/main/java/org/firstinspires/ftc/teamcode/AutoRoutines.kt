package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.extensions.pedro.FollowPath
import org.firstinspires.ftc.teamcode.subsystems.LimeLight

object AutoRoutines {

    val threeArtifactGoalStartAutoRoutine = SequentialGroup(
        FollowPath(TrajectoryFactory.goalStartToObelisk),
        LimeLight.detectMotif,
        FollowPath(TrajectoryFactory.obeliskToScore),
        Routines.motifShoot

    )
}
package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.BezierPoint
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathBuilder
import com.pedropathing.paths.PathChain
import dev.nextftc.core.units.deg
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.TrajectoryFactory.builder
import org.firstinspires.ftc.teamcode.pedroPathing.Constants


object TrajectoryFactory {

    //region Poses

    val goalStartPos = Pose(26.0, 130.0, -40.deg.inRad)

    val farStartPos = Pose(87.0, 9.0, -90.deg.inRad)

    val obeliskSensePos = /*Pose(0.0,0.0,0.0) / */ Pose(48.0, 115.0, -125.deg.inRad)

    val scorePos = Pose(40.0, 110.0, -36.deg.inRad)

    val spikeMark1PosPre = Pose(40.6, 83.6, 180.deg.inRad)

    val spikeMark1PosInner = Pose(35.6, 83.6, 180.deg.inRad)

    val spikeMark1PosMiddle = Pose(30.6, 83.6, 180.deg.inRad)

    val spikeMark1PosOuter = Pose(25.6, 83.6, 180.deg.inRad)

    val spikeMark2PosPre = Pose(40.6, 59.6, 180.deg.inRad)

    val spikeMark2PosInner = Pose(35.6, 59.6, 180.deg.inRad)

    val spikeMark2PosMiddle = Pose(30.6, 59.6, 180.deg.inRad)

    val spikeMark2PosOuter = Pose(25.6, 59.6, 180.deg.inRad)

    val spikeMark3PosPre = Pose(40.6, 35.6, 180.deg.inRad)

    val spikeMark3PosInner = Pose(35.6, 35.6, 180.deg.inRad)

    val spikeMark3PosMiddle = Pose(30.6, 35.6, 180.deg.inRad)

    val spikeMark3PosOuter = Pose(25.6, 35.6, 180.deg.inRad)

    //endregion

    //region Paths


    lateinit var builder: PathBuilder

    lateinit var goalStartToObelisk: PathChain

    lateinit var obeliskToScore: PathChain

    lateinit var scoreToSpikeMark1: PathChain

    lateinit var spikeMark1Pickup: PathChain

    lateinit var spikeMark1PickupToScore: PathChain

    lateinit var scoreToSpikeMark2: PathChain

    lateinit var spikeMark2Pickup: PathChain

    lateinit var spikeMark2PickupToScore: PathChain

    lateinit var scoreToSpikeMark3: PathChain

    lateinit var spikeMark3Pickup: PathChain

    lateinit var spikeMark3PickupToScore: PathChain


    fun buildTrajectories(follower: Follower) {
        builder = PathBuilder(follower)

        goalStartToObelisk = builder
                .addPath(BezierPoint(obeliskSensePos)).build()

        obeliskToScore = builder
                .addPath(BezierPoint(scorePos)).build()

        scoreToSpikeMark1 = builder
            .addPath(BezierPoint(spikeMark1PosPre)).build()

        spikeMark1Pickup = builder
            .addPath(BezierPoint(spikeMark1PosInner))
            .addPath(BezierPoint(spikeMark1PosMiddle))
            .addPath(BezierPoint(spikeMark1PosOuter)).build()

        spikeMark1PickupToScore = builder
            .addPath(BezierPoint(scorePos)).build()

        scoreToSpikeMark2 = builder
            .addPath(BezierPoint(spikeMark2PosPre)).build()

        spikeMark2Pickup = builder
            .addPath(BezierPoint(spikeMark2PosInner))
            .addPath(BezierPoint(spikeMark2PosMiddle))
            .addPath(BezierPoint(spikeMark2PosOuter)).build()

        spikeMark2PickupToScore = builder
            .addPath(BezierPoint(scorePos)).build()

        scoreToSpikeMark3 = builder
            .addPath(BezierPoint(spikeMark3PosPre)).build()

        spikeMark3Pickup = builder
            .addPath(BezierPoint(spikeMark3PosInner))
            .addPath(BezierPoint(spikeMark3PosMiddle))
            .addPath(BezierPoint(spikeMark3PosOuter)).build()

        spikeMark3PickupToScore = builder
            .addPath(BezierPoint(scorePos)).build()
    }


    //endregion
}
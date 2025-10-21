package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathBuilder
import com.pedropathing.paths.PathChain
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.TrajectoryFactory.builder
import org.firstinspires.ftc.teamcode.pedroPathing.Constants


object TrajectoryFactory {

    //region Poses

    val goalStartPos = Pose(20.3, 122.8, -36.0)

    val farStartPos = Pose(87.0, 9.0, -90.0)

    val obeliskSensePos = /*Pose(0.0,0.0,0.0) / */ Pose(40.0, 110.0, -135.0)

    val scorePos = Pose(40.0, 110.0, -36.0)

    val spikeMark1PosPre = Pose(40.6, 83.6, 180.0)

    val spikeMark1PosInner = Pose(35.6, 83.6, 180.0)

    val spikeMark1PosMiddle = Pose(30.6, 83.6, 180.0)

    val spikeMark1PosOuter = Pose(25.6, 83.6, 180.0)

    val spikeMark2PosPre = Pose(40.6, 59.6, 180.0)

    val spikeMark2PosInner = Pose(35.6, 59.6, 180.0)

    val spikeMark2PosMiddle = Pose(30.6, 59.6, 180.0)

    val spikeMark2PosOuter = Pose(25.6, 59.6, 180.0)

    val spikeMark3PosPre = Pose(40.6, 35.6, 180.0)

    val spikeMark3PosInner = Pose(35.6, 35.6, 180.0)

    val spikeMark3PosMiddle = Pose(30.6, 35.6, 180.0)

    val spikeMark3PosOuter = Pose(25.6, 35.6, 180.0)

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
                .addPath(BezierLine(goalStartPos, obeliskSensePos)).build()

        obeliskToScore = builder
                .addPath(BezierLine(obeliskSensePos, scorePos)).build()

        scoreToSpikeMark1 = builder
            .addPath(BezierLine(scorePos, spikeMark1PosPre)).build()

        spikeMark1Pickup = builder
            .addPath(BezierLine(spikeMark1PosPre, spikeMark1PosInner))
            .addPath(BezierLine(spikeMark1PosInner, spikeMark1PosMiddle))
            .addPath(BezierLine(spikeMark1PosMiddle, spikeMark1PosOuter)).build()

        spikeMark1PickupToScore = builder
            .addPath(BezierLine(spikeMark1PosOuter, scorePos)).build()

        scoreToSpikeMark2 = builder
            .addPath(BezierLine(scorePos, spikeMark2PosPre)).build()

        spikeMark2Pickup = builder
            .addPath(BezierLine(spikeMark2PosPre, spikeMark2PosInner))
            .addPath(BezierLine(spikeMark2PosInner, spikeMark2PosMiddle))
            .addPath(BezierLine(spikeMark2PosMiddle, spikeMark2PosOuter)).build()

        spikeMark2PickupToScore = builder
            .addPath(BezierLine(spikeMark2PosOuter, scorePos)).build()

        scoreToSpikeMark3 = builder
            .addPath(BezierLine(scorePos, spikeMark3PosPre)).build()

        spikeMark3Pickup = builder
            .addPath(BezierLine(spikeMark3PosPre, spikeMark3PosInner))
            .addPath(BezierLine(spikeMark3PosInner, spikeMark3PosMiddle))
            .addPath(BezierLine(spikeMark3PosMiddle, spikeMark3PosOuter)).build()

        spikeMark3PickupToScore = builder
            .addPath(BezierLine(spikeMark3PosOuter, scorePos)).build()
    }


    //endregion
}
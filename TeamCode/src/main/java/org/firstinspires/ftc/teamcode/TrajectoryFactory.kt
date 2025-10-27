package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.BezierPoint
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathBuilder
import com.pedropathing.paths.PathChain
import dev.nextftc.core.units.deg
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.pedroPathing.Constants


object TrajectoryFactory {

    //region Poses

    val goalStartPos = Pose(26.0, 130.0, -40.deg.inRad)

    val farStartPos = Pose(87.0, 9.0, -90.deg.inRad)

    val obeliskSensePos = /*Pose(0.0,0.0,0.0) / */ Pose(48.0, 115.0, -125.deg.inRad)

    val scorePos = Pose(31.0, 120.0, -42.deg.inRad)

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

    val outOfTheWayPos = Pose(40.0, 136.0, -90.deg.inRad)

    //endregion

    //region Paths

    lateinit var goalStartToObelisk: PathChain

    lateinit var obeliskToScore: PathChain

    lateinit var scoreToSpikeMark1: PathChain

    lateinit var spikeMark1Pickup1: PathChain
    lateinit var spikeMark1Pickup2: PathChain
    lateinit var spikeMark1Pickup3: PathChain

    lateinit var spikeMark1PickupToScore: PathChain

    lateinit var scoreToSpikeMark2: PathChain

    lateinit var spikeMark2Pickup1: PathChain
    lateinit var spikeMark2Pickup2: PathChain
    lateinit var spikeMark2Pickup3: PathChain

    lateinit var spikeMark2PickupToScore: PathChain

    lateinit var scoreToSpikeMark3: PathChain

    lateinit var spikeMark3Pickup1: PathChain
    lateinit var spikeMark3Pickup2: PathChain
    lateinit var spikeMark3Pickup3: PathChain

    lateinit var spikeMark3PickupToScore: PathChain

    lateinit var scoreToOutOfTheWay: PathChain


    fun buildTrajectories(follower: Follower) {
        goalStartToObelisk = follower.pathBuilder()
                .addPath(BezierLine(goalStartPos, obeliskSensePos))
                .setLinearHeadingInterpolation(goalStartPos.heading, obeliskSensePos.heading).build()

        obeliskToScore = follower.pathBuilder()
                .addPath(BezierLine(obeliskSensePos, scorePos))
                .setLinearHeadingInterpolation(obeliskSensePos.heading, scorePos.heading)
                .build()

        scoreToSpikeMark1 = follower.pathBuilder()
            .addPath(BezierCurve(scorePos, Pose(spikeMark1PosPre.x + 25, spikeMark1PosPre.y, spikeMark1PosPre.heading), spikeMark1PosPre))
            .setLinearHeadingInterpolation(scorePos.heading, spikeMark1PosPre.heading)
            .build()


        spikeMark1Pickup1 = follower.pathBuilder()
            .addPath(BezierLine(spikeMark1PosPre, Pose(spikeMark1PosInner.x - 0.5, spikeMark1PosInner.y, spikeMark1PosInner.heading)))
            .addPath(BezierLine(Pose(spikeMark1PosInner.x - 0.5, spikeMark1PosInner.y, spikeMark1PosInner.heading), spikeMark1PosInner))
            .setLinearHeadingInterpolation(spikeMark1PosPre.heading, spikeMark1PosOuter.heading)
            .build()

        /*
        spikeMark1Pickup2 = follower.pathBuilder()
            .addPath(BezierLine(spikeMark1PosInner, Pose(spikeMark1PosMiddle.x - 0.5, spikeMark1PosMiddle.y, spikeMark1PosMiddle.heading)))
            .addPath(BezierLine(Pose(spikeMark1PosMiddle.x - 0.5, spikeMark1PosMiddle.y, spikeMark1PosInner.heading), spikeMark1PosMiddle))
            .setLinearHeadingInterpolation(spikeMark1PosInner.heading, spikeMark1PosMiddle.heading)
            .build()

        spikeMark1Pickup3 = follower.pathBuilder()
            .addPath(BezierLine(spikeMark1PosMiddle, Pose(spikeMark1PosOuter.x - 0.5, spikeMark1PosOuter.y, spikeMark1PosOuter.heading)))
            .addPath(BezierLine(Pose(spikeMark1PosOuter.x - 0.5, spikeMark1PosOuter.y, spikeMark1PosOuter.heading), spikeMark1PosOuter))
            .setLinearHeadingInterpolation(spikeMark1PosMiddle.heading, spikeMark1PosOuter.heading)
            .build()

         */

//        spikeMark1Pickup1 = follower.pathBuilder()
//            .addPath(BezierLine(spikeMark1PosPre, spikeMark1PosInner))
//            .setLinearHeadingInterpolation(spikeMark1PosPre.heading, spikeMark1PosOuter.heading)
//            .build()

        spikeMark1Pickup2 = follower.pathBuilder()
            .addPath(BezierLine(spikeMark1PosInner, spikeMark1PosMiddle))
            .setLinearHeadingInterpolation(spikeMark1PosInner.heading, spikeMark1PosMiddle.heading)
            .build()

        spikeMark1Pickup3 = follower.pathBuilder()
            .addPath(BezierLine(spikeMark1PosMiddle, spikeMark1PosOuter))
            .setLinearHeadingInterpolation(spikeMark1PosMiddle.heading, spikeMark1PosOuter.heading)
            .build()

        spikeMark1PickupToScore = follower.pathBuilder()
            .addPath(BezierLine(spikeMark1PosOuter, scorePos))
            .setLinearHeadingInterpolation(spikeMark1PosOuter.heading, scorePos.heading)
            .build()

        scoreToSpikeMark2 = follower.pathBuilder()
            .addPath(BezierLine(scorePos, spikeMark2PosPre))
            .setLinearHeadingInterpolation(scorePos.heading, spikeMark2PosPre.heading)
            .build()

        spikeMark2Pickup1 = follower.pathBuilder()
            .addPath(BezierLine(spikeMark2PosPre, spikeMark2PosInner))
            .setLinearHeadingInterpolation(spikeMark2PosPre.heading, spikeMark2PosOuter.heading)
            .build()

        spikeMark2Pickup2 = follower.pathBuilder()
            .addPath(BezierLine(spikeMark2PosInner, spikeMark2PosMiddle))
            .setLinearHeadingInterpolation(spikeMark2PosInner.heading, spikeMark2PosMiddle.heading)
            .build()

        spikeMark2Pickup3 = follower.pathBuilder()
            .addPath(BezierLine(spikeMark2PosMiddle, spikeMark2PosOuter))
            .setLinearHeadingInterpolation(spikeMark2PosMiddle.heading, spikeMark2PosOuter.heading)
            .build()

        spikeMark2PickupToScore = follower.pathBuilder()
            .addPath(BezierLine(spikeMark2PosOuter, scorePos))
            .setLinearHeadingInterpolation(spikeMark2PosOuter.heading, scorePos.heading)
            .build()

        scoreToSpikeMark3 = follower.pathBuilder()
            .addPath(BezierLine(scorePos, spikeMark3PosPre))
            .setLinearHeadingInterpolation(scorePos.heading, spikeMark3PosPre.heading)
            .build()

        spikeMark3Pickup1 = follower.pathBuilder()
            .addPath(BezierLine(spikeMark3PosPre, spikeMark3PosInner))
            .setLinearHeadingInterpolation(spikeMark3PosPre.heading, spikeMark3PosOuter.heading)
            .build()

        spikeMark3Pickup2 = follower.pathBuilder()
            .addPath(BezierLine(spikeMark3PosInner, spikeMark3PosMiddle))
            .setLinearHeadingInterpolation(spikeMark3PosInner.heading, spikeMark3PosMiddle.heading)
            .build()

        spikeMark3Pickup3 = follower.pathBuilder()
            .addPath(BezierLine(spikeMark3PosMiddle, spikeMark3PosOuter))
            .setLinearHeadingInterpolation(spikeMark3PosMiddle.heading, spikeMark3PosOuter.heading)
            .build()

        spikeMark3PickupToScore = follower.pathBuilder()
            .addPath(BezierLine(spikeMark3PosOuter, scorePos))
            .setLinearHeadingInterpolation(spikeMark3PosOuter.heading, scorePos.heading)
            .build()

        scoreToOutOfTheWay = follower.pathBuilder()
            .addPath(BezierLine(scorePos, outOfTheWayPos))
            .setLinearHeadingInterpolation(scorePos.heading, outOfTheWayPos.heading)
            .build()
    }


    //endregion
}
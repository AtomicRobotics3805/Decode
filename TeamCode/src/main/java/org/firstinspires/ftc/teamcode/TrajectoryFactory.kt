package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import dev.nextftc.core.units.deg
import org.firstinspires.ftc.teamcode.autos.AutonomousInfo


object TrajectoryFactory {

    //region Poses

    val goalStartPos = Pose(26.0, 130.0, -40.deg.inRad)

    val farStartPos = Pose(56.0, 8.0, 180.deg.inRad)

    val farParkPos = Pose(36.0, 8.0, 180.deg.inRad)

    val obeliskSensePos = /*Pose(0.0,0.0,0.0) / */ Pose(48.0, 115.0, -125.deg.inRad)

    val scorePos = Pose(30.0, 119.0, -42.deg.inRad)

    val spikeMark1PosPre = Pose(41.6, 83.6, 180.deg.inRad)

    val spikeMark1PosInner = Pose(36.75, 83.6, 180.deg.inRad)

    val spikeMark1PosMiddle = Pose(31.6, 83.6, 180.deg.inRad)

    val spikeMark1PosOuter = Pose(26.6, 83.6, 180.deg.inRad)

    val spikeMark2PosPre = Pose(41.6, 60.0, 180.deg.inRad)

    val spikeMark2PosInner = Pose(36.75, 60.0, 180.deg.inRad)

    val spikeMark2PosMiddle = Pose(31.6, 60.0, 180.deg.inRad)

    val spikeMark2PosOuter = Pose(26.6, 60.0, 180.deg.inRad)

    val spikeMark3PosPre = Pose(40.6, 35.6, 180.deg.inRad)

    val spikeMark3PosInner = Pose(35.6, 35.6, 180.deg.inRad)

    val spikeMark3PosMiddle = Pose(30.6, 35.6, 180.deg.inRad)

    val spikeMark3PosOuter = Pose(25.6, 35.6, 180.deg.inRad)

    val outOfTheWayPos = Pose(45.0, 128.0, 180.deg.inRad)

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

    lateinit var farStartToPark: PathChain

    fun flipPaths() {

    }

    fun buildTrajectories(follower: Follower) {

        if (AutonomousInfo.redAuto) {
            goalStartToObelisk = follower.pathBuilder()
                .addPath(BezierLine(goalStartPos.mirror(), obeliskSensePos.mirror()))
                .setLinearHeadingInterpolation(goalStartPos.mirror().heading, obeliskSensePos.mirror().heading)
                .build()

            obeliskToScore = follower.pathBuilder()
                .addPath(BezierLine(obeliskSensePos.mirror(), Pose(scorePos.mirror().x, scorePos.mirror().y - 2, scorePos.mirror().heading)))
                .setLinearHeadingInterpolation(obeliskSensePos.mirror().heading, scorePos.mirror().heading)
                .build()

            scoreToSpikeMark1 = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        Pose(scorePos.mirror().x, scorePos.mirror().y - 2, scorePos.mirror().heading),
                        Pose(spikeMark1PosPre.x + 30, spikeMark1PosPre.y, spikeMark1PosPre.heading).mirror(),
                        spikeMark1PosPre.mirror()
                    )
                )
                .setLinearHeadingInterpolation(scorePos.mirror().heading, spikeMark1PosPre.mirror().heading)
                .build()


            spikeMark1Pickup1 = follower.pathBuilder()
                .addPath(BezierLine(spikeMark1PosPre.mirror(), spikeMark1PosInner.mirror()))
                .setLinearHeadingInterpolation(spikeMark1PosPre.mirror().heading, spikeMark1PosInner.mirror().heading)
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
                .addPath(BezierLine(spikeMark1PosInner.mirror(), spikeMark1PosMiddle.mirror()))
                .setLinearHeadingInterpolation(
                    spikeMark1PosInner.mirror().heading,
                    spikeMark1PosMiddle.mirror().heading
                )
                .build()

            spikeMark1Pickup3 = follower.pathBuilder()
                .addPath(BezierLine(spikeMark1PosMiddle.mirror(), spikeMark1PosOuter.mirror()))
                .setLinearHeadingInterpolation(
                    spikeMark1PosMiddle.mirror().heading,
                    spikeMark1PosOuter.mirror().heading
                )
                .build()

            spikeMark1PickupToScore = follower.pathBuilder()
                .addPath(BezierLine(spikeMark1PosOuter.mirror(), scorePos.mirror()))
                .setLinearHeadingInterpolation(spikeMark1PosOuter.mirror().heading, scorePos.mirror().heading)
                .build()

            scoreToSpikeMark2 = follower.pathBuilder()
                .addPath(BezierCurve(
                    scorePos.mirror(),
                    Pose(spikeMark2PosPre.x + 25, spikeMark2PosPre.y, spikeMark2PosPre.heading).mirror(),
                    spikeMark2PosPre.mirror()))
                .setLinearHeadingInterpolation(scorePos.mirror().heading, spikeMark2PosPre.mirror().heading)
                .build()

            spikeMark2Pickup1 = follower.pathBuilder()
                .addPath(BezierLine(spikeMark2PosPre.mirror(), spikeMark2PosInner.mirror()))
                .setLinearHeadingInterpolation(spikeMark2PosPre.mirror().heading, spikeMark2PosOuter.mirror().heading)
                .build()

            spikeMark2Pickup2 = follower.pathBuilder()
                .addPath(BezierLine(spikeMark2PosInner.mirror(), spikeMark2PosMiddle.mirror()))
                .setLinearHeadingInterpolation(
                    spikeMark2PosInner.mirror().heading,
                    spikeMark2PosMiddle.mirror().heading
                )
                .build()

            spikeMark2Pickup3 = follower.pathBuilder()
                .addPath(BezierLine(spikeMark2PosMiddle.mirror(), spikeMark2PosOuter.mirror()))
                .setLinearHeadingInterpolation(
                    spikeMark2PosMiddle.mirror().heading,
                    spikeMark2PosOuter.mirror().heading
                )
                .build()

            spikeMark2PickupToScore = follower.pathBuilder()
                .addPath(BezierLine(spikeMark2PosOuter.mirror(), scorePos.mirror()))
                .setLinearHeadingInterpolation(spikeMark2PosOuter.mirror().heading, scorePos.mirror().heading)
                .build()

            scoreToSpikeMark3 = follower.pathBuilder()
                .addPath(BezierLine(scorePos.mirror(), spikeMark3PosPre.mirror()))
                .setLinearHeadingInterpolation(scorePos.mirror().heading, spikeMark3PosPre.mirror().heading)
                .build()

            spikeMark3Pickup1 = follower.pathBuilder()
                .addPath(BezierLine(spikeMark3PosPre.mirror(), spikeMark3PosInner.mirror()))
                .setLinearHeadingInterpolation(spikeMark3PosPre.mirror().heading, spikeMark3PosOuter.mirror().heading)
                .build()

            spikeMark3Pickup2 = follower.pathBuilder()
                .addPath(BezierLine(spikeMark3PosInner.mirror(), spikeMark3PosMiddle.mirror()))
                .setLinearHeadingInterpolation(
                    spikeMark3PosInner.mirror().heading,
                    spikeMark3PosMiddle.mirror().heading
                )
                .build()

            spikeMark3Pickup3 = follower.pathBuilder()
                .addPath(BezierLine(spikeMark3PosMiddle.mirror(), spikeMark3PosOuter.mirror()))
                .setLinearHeadingInterpolation(
                    spikeMark3PosMiddle.mirror().heading,
                    spikeMark3PosOuter.mirror().heading
                )
                .build()

            spikeMark3PickupToScore = follower.pathBuilder()
                .addPath(BezierLine(spikeMark3PosOuter.mirror(), scorePos.mirror()))
                .setLinearHeadingInterpolation(spikeMark3PosOuter.mirror().heading, scorePos.mirror().heading)
                .build()

            scoreToOutOfTheWay = follower.pathBuilder()
                .addPath(BezierLine(scorePos.mirror(), outOfTheWayPos.mirror()))
                .setLinearHeadingInterpolation(scorePos.mirror().heading, outOfTheWayPos.mirror().heading)
                .build()

            farStartToPark = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        farStartPos.mirror(),
                        Pose(42.000, 15.000).mirror(),
                        farParkPos.mirror()
                    ))
                .setLinearHeadingInterpolation(farStartPos.mirror().heading, farParkPos.mirror().heading)
                .build()
        }
        else {
            goalStartToObelisk = follower.pathBuilder()
                .addPath(BezierLine(goalStartPos, obeliskSensePos))
                .setLinearHeadingInterpolation(goalStartPos.heading, obeliskSensePos.heading)
                .build()

            obeliskToScore = follower.pathBuilder()
                .addPath(BezierLine(obeliskSensePos, scorePos))
                .setLinearHeadingInterpolation(obeliskSensePos.heading, scorePos.heading)
                .build()

            scoreToSpikeMark1 = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        scorePos,
                        Pose(spikeMark1PosPre.x + 25, spikeMark1PosPre.y, spikeMark1PosPre.heading),
                        spikeMark1PosPre
                    )
                )
                .setLinearHeadingInterpolation(scorePos.heading, spikeMark1PosPre.heading)
                .build()


            spikeMark1Pickup1 = follower.pathBuilder()
                .addPath(BezierLine(spikeMark1PosPre, spikeMark1PosInner))
                .setLinearHeadingInterpolation(spikeMark1PosPre.heading, spikeMark1PosInner.heading)
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
                .setLinearHeadingInterpolation(
                    spikeMark1PosInner.heading,
                    spikeMark1PosMiddle.heading
                )
                .build()

            spikeMark1Pickup3 = follower.pathBuilder()
                .addPath(BezierLine(spikeMark1PosMiddle, spikeMark1PosOuter))
                .setLinearHeadingInterpolation(
                    spikeMark1PosMiddle.heading,
                    spikeMark1PosOuter.heading
                )
                .build()

            spikeMark1PickupToScore = follower.pathBuilder()
                .addPath(BezierLine(spikeMark1PosOuter, scorePos))
                .setLinearHeadingInterpolation(spikeMark1PosOuter.heading, scorePos.heading)
                .build()

            scoreToSpikeMark2 = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        scorePos,
                        Pose(spikeMark2PosPre.x + 25, spikeMark2PosPre.y, spikeMark2PosPre.heading),
                        spikeMark2PosPre)
                )
                .setLinearHeadingInterpolation(scorePos.heading, spikeMark2PosPre.heading)
                .build()

            spikeMark2Pickup1 = follower.pathBuilder()
                .addPath(BezierLine(spikeMark2PosPre, spikeMark2PosInner))
                .setLinearHeadingInterpolation(spikeMark2PosPre.heading, spikeMark2PosOuter.heading)
                .build()

            spikeMark2Pickup2 = follower.pathBuilder()
                .addPath(BezierLine(spikeMark2PosInner, spikeMark2PosMiddle))
                .setLinearHeadingInterpolation(
                    spikeMark2PosInner.heading,
                    spikeMark2PosMiddle.heading
                )
                .build()

            spikeMark2Pickup3 = follower.pathBuilder()
                .addPath(BezierLine(spikeMark2PosMiddle, spikeMark2PosOuter))
                .setLinearHeadingInterpolation(
                    spikeMark2PosMiddle.heading,
                    spikeMark2PosOuter.heading
                )
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
                .setLinearHeadingInterpolation(
                    spikeMark3PosInner.heading,
                    spikeMark3PosMiddle.heading
                )
                .build()

            spikeMark3Pickup3 = follower.pathBuilder()
                .addPath(BezierLine(spikeMark3PosMiddle, spikeMark3PosOuter))
                .setLinearHeadingInterpolation(
                    spikeMark3PosMiddle.heading,
                    spikeMark3PosOuter.heading
                )
                .build()

            spikeMark3PickupToScore = follower.pathBuilder()
                .addPath(BezierLine(spikeMark3PosOuter, scorePos))
                .setLinearHeadingInterpolation(spikeMark3PosOuter.heading, scorePos.heading)
                .build()

            scoreToOutOfTheWay = follower.pathBuilder()
                .addPath(BezierLine(scorePos, outOfTheWayPos))
                .setLinearHeadingInterpolation(scorePos.heading, outOfTheWayPos.heading)
                .build()

            farStartToPark = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        farStartPos,
                        Pose(42.000, 15.000),
                        farParkPos
                    ))
                .setLinearHeadingInterpolation(farStartPos.heading, farParkPos.heading)
                .build()
        }
    }


    //endregion
}
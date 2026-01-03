package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import org.firstinspires.ftc.teamcode.autos.AutonomousInfo


object TrajectoryFactory {

    val redPickupOffset = 5.0

    val bluePickupOffset = 2.0

    //region Poses

    val goalStartPos = Pose(28.0, 130.0, (-40 + 90).deg.inRad)

    val farStartPos = Pose(57.0, 9.0, 270.deg.inRad)

    val farParkPos = Pose(57.0, 29.0, 180.deg.inRad)

    val farShootPos = Pose(58.5, 11.0, 1.9742947094299996.rad.inRad)

    val obeliskSensePos = /*Pose(0.0,0.0,0.0) / */ Pose(48.0, 115.0, 55.deg.inRad)

    val scorePos = Pose(61.0, 102.5, 146.deg.inRad)

    val spikeMark1PosPre = Pose(41.6, 83.1, 180.deg.inRad)

    val spikeMark1PosInner = Pose(36.6, 83.1, 180.deg.inRad)

    val spikeMark1PosMiddle = Pose(31.6, 83.1, 180.deg.inRad)

    val spikeMark1PosOuter = Pose(26.6, 83.1, 180.deg.inRad)

    val spikeMark2PosPre = Pose(40.6, 59.0, 180.deg.inRad)

    val spikeMark2PosInner = Pose(36.75, 59.0, 180.deg.inRad)

    val spikeMark2PosMiddle = Pose(31.6, 59.0, 180.deg.inRad)

    val spikeMark2PosOuter = Pose(26.6, 59.0, 180.deg.inRad)

    val dumpPos = Pose(14.5, 77.0, 180.deg.inRad)

    val spikeMark3PosPre = Pose(40.6, 35.6, 180.deg.inRad)

    val spikeMark3PosInner = Pose(35.6, 35.6, 180.deg.inRad)

    val spikeMark3PosMiddle = Pose(30.6, 35.6, 180.deg.inRad)

    val spikeMark3PosOuter = Pose(25.6, 35.6, 180.deg.inRad)

    val outOfTheWayPos = Pose(41.0, 70.0, 180.deg.inRad)

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

    lateinit var spikeMark1ToDump: PathChain
    lateinit var spikeMark2ToDump: PathChain

    lateinit var dumpToScore: PathChain

    lateinit var spikeMark2PickupToScore: PathChain

    lateinit var scoreToSpikeMark3: PathChain

    lateinit var spikeMark3Pickup1: PathChain
    lateinit var spikeMark3Pickup2: PathChain
    lateinit var spikeMark3Pickup3: PathChain

    lateinit var spikeMark3PickupToScore: PathChain

    lateinit var scoreToOutOfTheWay: PathChain

    lateinit var farStartToPark: PathChain

    lateinit var farStartToShoot: PathChain

    lateinit var farShootPosToPark: PathChain


    fun buildTrajectories(follower: Follower) {

        if (AutonomousInfo.redAuto) {
            goalStartToObelisk = follower.pathBuilder()
                .addPath(BezierLine(goalStartPos.mirror(), obeliskSensePos.mirror()))
                .setLinearHeadingInterpolation(goalStartPos.mirror().heading, obeliskSensePos.mirror().heading)
                .build()

            obeliskToScore = follower.pathBuilder()
                .addPath(BezierLine(obeliskSensePos.mirror(), Pose(scorePos.x, scorePos.y - 2, scorePos.heading).mirror()))
                .setLinearHeadingInterpolation(obeliskSensePos.mirror().heading, scorePos.mirror().heading)
                .build()

            scoreToSpikeMark1 = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        Pose(scorePos.mirror().x, scorePos.mirror().y - 2, scorePos.mirror().heading),
                        Pose(spikeMark1PosPre.x + 30, spikeMark1PosPre.y, spikeMark1PosPre.heading).mirror(),
                        Pose(spikeMark1PosPre.x - redPickupOffset, spikeMark1PosPre.y, spikeMark1PosPre.heading).mirror()
                    )
                )
                .setLinearHeadingInterpolation(scorePos.mirror().heading, spikeMark1PosPre.mirror().heading)
                .build()


            spikeMark1Pickup1 = follower.pathBuilder()
                .addPath(BezierLine(Pose(spikeMark1PosPre.x - redPickupOffset, spikeMark1PosPre.y, spikeMark1PosPre.heading).mirror(),
                    Pose(spikeMark1PosInner.x - redPickupOffset, spikeMark1PosInner.y, spikeMark1PosInner.heading).mirror()))
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
                .addPath(BezierLine(Pose(spikeMark1PosInner.x - redPickupOffset, spikeMark1PosInner.y, spikeMark1PosInner.heading).mirror(),
                    Pose(spikeMark1PosMiddle.x - redPickupOffset, spikeMark1PosMiddle.y, spikeMark1PosMiddle.heading).mirror()))
                .setLinearHeadingInterpolation(
                    spikeMark1PosInner.mirror().heading,
                    spikeMark1PosMiddle.mirror().heading
                )
                .build()

            spikeMark1Pickup3 = follower.pathBuilder()
                .addPath(BezierLine(Pose(spikeMark1PosMiddle.x - redPickupOffset, spikeMark1PosMiddle.y, spikeMark1PosMiddle.heading).mirror(),
                    Pose(spikeMark1PosOuter.x - redPickupOffset, spikeMark1PosOuter.y, spikeMark1PosOuter.heading).mirror()))
                .setLinearHeadingInterpolation(
                    spikeMark1PosMiddle.mirror().heading,
                    spikeMark1PosOuter.mirror().heading
                )
                .build()

            spikeMark1PickupToScore = follower.pathBuilder()
                .addPath(BezierLine(Pose(spikeMark1PosOuter.x - redPickupOffset, spikeMark1PosOuter.y, spikeMark1PosOuter.heading).mirror(),
                    scorePos.mirror()))
                .setLinearHeadingInterpolation(spikeMark1PosOuter.mirror().heading, scorePos.mirror().heading)
                .build()

            scoreToSpikeMark2 = follower.pathBuilder()
                .addPath(BezierCurve(scorePos.mirror(),
                    Pose(spikeMark2PosPre.x + 25, spikeMark2PosPre.y, spikeMark2PosPre.heading).mirror(),
                    Pose(spikeMark2PosPre.x - redPickupOffset, spikeMark2PosPre.y, spikeMark2PosPre.heading).mirror()))
                .setLinearHeadingInterpolation(scorePos.mirror().heading, spikeMark2PosPre.mirror().heading)
                .build()

            spikeMark2Pickup1 = follower.pathBuilder()
                .addPath(BezierLine(Pose(spikeMark2PosPre.x - redPickupOffset, spikeMark2PosPre.y, spikeMark2PosPre.heading).mirror(),
                    Pose(spikeMark2PosInner.x - redPickupOffset, spikeMark2PosInner.y, spikeMark2PosInner.heading).mirror()))
                .setLinearHeadingInterpolation(spikeMark2PosPre.mirror().heading, spikeMark2PosOuter.mirror().heading)
                .build()

            spikeMark2Pickup2 = follower.pathBuilder()
                .addPath(BezierLine(Pose(spikeMark2PosInner.x - redPickupOffset, spikeMark2PosInner.y, spikeMark2PosInner.heading).mirror(),
                    Pose(spikeMark2PosMiddle.x - redPickupOffset, spikeMark2PosMiddle.y, spikeMark2PosMiddle.heading).mirror()))
                .setLinearHeadingInterpolation(
                    spikeMark2PosInner.mirror().heading,
                    spikeMark2PosMiddle.mirror().heading
                )
                .build()

            spikeMark2Pickup3 = follower.pathBuilder()
                .addPath(BezierLine(Pose(spikeMark2PosMiddle.x - redPickupOffset, spikeMark2PosMiddle.y, spikeMark2PosMiddle.heading).mirror(),
                    Pose(spikeMark2PosOuter.x - redPickupOffset, spikeMark2PosOuter.y, spikeMark2PosOuter.heading).mirror()))
                .setLinearHeadingInterpolation(
                    spikeMark2PosMiddle.mirror().heading,
                    spikeMark2PosOuter.mirror().heading
                )
                .build()

            spikeMark1ToDump = follower.pathBuilder()
                .addPath(BezierCurve(Pose(spikeMark1PosOuter.x - redPickupOffset, spikeMark1PosOuter.y, spikeMark1PosOuter.heading).mirror(),
                    Pose(28.5, 75.5).mirror(),
                    dumpPos.mirror()))
                .setLinearHeadingInterpolation(spikeMark1PosOuter.mirror().heading, dumpPos.mirror().heading)
                .build()

            spikeMark2ToDump = follower.pathBuilder()
                .addPath(BezierCurve(Pose(spikeMark2PosOuter.x - redPickupOffset, spikeMark2PosOuter.y, spikeMark2PosOuter.heading).mirror(),
                    Pose(40.0, 73.0).mirror(),
                    dumpPos.mirror()))
                .setLinearHeadingInterpolation(spikeMark2PosOuter.mirror().heading, dumpPos.mirror().heading)
                .build()

            dumpToScore = follower.pathBuilder()
                .addPath(BezierCurve(dumpPos.mirror(),
                    Pose(40.0, 71.0, scorePos.heading).mirror(),
                    scorePos.mirror()))
                .setLinearHeadingInterpolation(spikeMark2PosOuter.mirror().heading, scorePos.mirror().heading)
                .build()

            spikeMark2PickupToScore = follower.pathBuilder()
                .addPath(BezierCurve(Pose(spikeMark2PosOuter.x - redPickupOffset, spikeMark2PosOuter.y, spikeMark2PosOuter.heading).mirror(),
                    Pose(33.0, 71.0, scorePos.heading).mirror(),
                    scorePos.mirror()))
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

            farStartToShoot = follower.pathBuilder()
                .addPath(BezierCurve(
                    farStartPos.mirror(),
                    Pose(68.0, 137.0).mirror(),
                    farShootPos.mirror()
                ))
                .setLinearHeadingInterpolation(farStartPos.mirror().heading, farShootPos.mirror().heading)
                .build()

            farShootPosToPark = follower.pathBuilder()
                .addPath(BezierCurve(
                    farShootPos.mirror(),
                    Pose(68.0, 137.0).mirror(),
                    farParkPos.mirror()
                ))
                .setLinearHeadingInterpolation(farShootPos.mirror().heading, farParkPos.mirror().heading)
                .build()

            farStartToPark = follower.pathBuilder()
                .addPath(BezierLine(farStartPos.mirror(), farParkPos.mirror()))
                .setLinearHeadingInterpolation(farStartPos.mirror().heading, farParkPos.mirror().heading)
                .build()
        } else {
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
                        Pose(spikeMark1PosPre.x - bluePickupOffset, spikeMark1PosPre.y, spikeMark1PosPre.heading)
                        // 
                    )
                )
                .setLinearHeadingInterpolation(scorePos.heading, spikeMark1PosPre.heading)
                .build()


            spikeMark1Pickup1 = follower.pathBuilder()
                .addPath(BezierLine(

                    Pose(spikeMark1PosPre.x - bluePickupOffset, spikeMark1PosPre.y, spikeMark1PosPre.heading),
                    Pose(spikeMark1PosInner.x - bluePickupOffset, spikeMark1PosInner.y, spikeMark1PosInner.heading)))
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
                .addPath(BezierLine(
                    Pose(spikeMark1PosInner.x - bluePickupOffset, spikeMark1PosInner.y, spikeMark1PosInner.heading),
                    Pose(spikeMark1PosMiddle.x - bluePickupOffset, spikeMark1PosMiddle.y, spikeMark1PosMiddle.heading)))
                .setLinearHeadingInterpolation(
                    spikeMark1PosInner.heading,
                    spikeMark1PosMiddle.heading
                )
                .build()

            spikeMark1Pickup3 = follower.pathBuilder()
                .addPath(BezierLine(Pose(spikeMark1PosMiddle.x - bluePickupOffset, spikeMark1PosMiddle.y, spikeMark1PosMiddle.heading),
                    Pose(spikeMark1PosOuter.x - bluePickupOffset, spikeMark1PosOuter.y, spikeMark1PosOuter.heading)))
                .setLinearHeadingInterpolation(
                    spikeMark1PosMiddle.heading,
                    spikeMark1PosOuter.heading
                )
                .build()

            spikeMark1PickupToScore = follower.pathBuilder()
                .addPath(BezierLine(Pose(spikeMark1PosOuter.x - bluePickupOffset, spikeMark1PosOuter.y, spikeMark1PosOuter.heading),
                    scorePos))
                .setLinearHeadingInterpolation(spikeMark1PosOuter.heading, scorePos.heading)
                .build()

            scoreToSpikeMark2 = follower.pathBuilder()
                .addPath(BezierCurve(scorePos,
                    Pose(spikeMark2PosPre.x + 25, spikeMark2PosPre.y, spikeMark2PosPre.heading),
                    Pose(spikeMark2PosPre.x - bluePickupOffset, spikeMark2PosPre.y, spikeMark2PosPre.heading)))
                .setLinearHeadingInterpolation(scorePos.heading, spikeMark2PosPre.heading)
                .build()

            spikeMark2Pickup1 = follower.pathBuilder()
                .addPath(BezierLine(Pose(spikeMark2PosPre.x - bluePickupOffset, spikeMark2PosPre.y, spikeMark2PosPre.heading),
                    Pose(spikeMark2PosInner.x - bluePickupOffset, spikeMark2PosInner.y, spikeMark2PosInner.heading)))
                .setLinearHeadingInterpolation(spikeMark2PosPre.heading, spikeMark2PosInner.heading)
                .build()

            spikeMark2Pickup2 = follower.pathBuilder()
                .addPath(BezierLine(Pose(spikeMark2PosInner.x - bluePickupOffset, spikeMark2PosInner.y, spikeMark2PosInner.heading),
                    Pose(spikeMark2PosMiddle.x - bluePickupOffset, spikeMark2PosMiddle.y, spikeMark2PosMiddle.heading)))
                .setLinearHeadingInterpolation(
                    spikeMark2PosInner.heading,
                    spikeMark2PosMiddle.heading
                )
                .build()

            spikeMark2Pickup3 = follower.pathBuilder()
                .addPath(BezierLine(Pose(spikeMark2PosMiddle.x - bluePickupOffset, spikeMark2PosMiddle.y, spikeMark2PosMiddle.heading),
                    Pose(spikeMark2PosOuter.x - bluePickupOffset, spikeMark2PosOuter.y, spikeMark2PosOuter.heading)))
                .setLinearHeadingInterpolation(
                    spikeMark2PosMiddle.heading,
                    spikeMark2PosOuter.heading
                )
                .build()


            spikeMark1ToDump = follower.pathBuilder()
                .addPath(BezierCurve(Pose(spikeMark1PosOuter.x - redPickupOffset, spikeMark1PosOuter.y, spikeMark1PosOuter.heading),
                    Pose(28.5, 75.5),
                    dumpPos))
                .setLinearHeadingInterpolation(spikeMark1PosOuter.heading, dumpPos.heading)
                .build()


            spikeMark2ToDump = follower.pathBuilder()
                .addPath(BezierCurve(Pose(spikeMark2PosOuter.x - bluePickupOffset, spikeMark2PosOuter.y, spikeMark2PosOuter.heading),
                    Pose(40.0, 73.0),
                    dumpPos))
                .setLinearHeadingInterpolation(spikeMark2PosOuter.heading, dumpPos.heading)
                .build()

            dumpToScore = follower.pathBuilder()
                .addPath(BezierCurve(dumpPos,
                    Pose(40.0, 71.0, scorePos.heading),
                    scorePos))
                .setLinearHeadingInterpolation(spikeMark2PosOuter.heading, scorePos.heading)
                .build()

            spikeMark2PickupToScore = follower.pathBuilder()
                .addPath(BezierCurve(dumpPos,
                    Pose(33.0, 71.0, scorePos.heading),
                    scorePos))
                .setLinearHeadingInterpolation(dumpPos.heading, scorePos.heading)
                .build()

            spikeMark2PickupToScore = follower.pathBuilder()
                .addPath(BezierCurve(Pose(spikeMark2PosOuter.x - bluePickupOffset, spikeMark2PosOuter.y, spikeMark2PosOuter.heading),
                    Pose(33.0, 71.0, scorePos.heading),
                    scorePos))
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

            farStartToShoot = follower.pathBuilder()
                .addPath(BezierCurve(
                    farStartPos,
                    Pose(68.0, 137.0),
                    farShootPos
                ))
                .setLinearHeadingInterpolation(farStartPos.heading, farShootPos.heading)
                .build()

            farShootPosToPark = follower.pathBuilder()
                .addPath(BezierCurve(
                    farShootPos,
                    Pose(68.0, 137.0),
                    farParkPos
                ))
                .setLinearHeadingInterpolation(farShootPos.heading, farParkPos.heading)
                .build()

            farStartToPark = follower.pathBuilder()
                .addPath(BezierLine(farStartPos, farParkPos))
                .setLinearHeadingInterpolation(farStartPos.heading, farParkPos.heading)
                .build()
        }
    }


    //endregion
}
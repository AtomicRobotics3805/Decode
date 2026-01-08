package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import org.firstinspires.ftc.teamcode.autos.AutonomousInfo


object TrajectoryFactory {

    val redPickupOffset = 5.0

    val bluePickupOffset = 2.0

    //region Poses

    val goalStartPos = Pose(28.0, 130.0, (-40 + 90).deg.inRad)

    val farStartPos = Pose(57.0, 9.0, 90.deg.inRad)

    val farShootPos = Pose(58.5, 13.0, 112.deg.inRad)

    val farParkPos = Pose(20.0, 9.0, 180.deg.inRad)

    val goalZoneParkPos = Pose(61.0, 102.0, 180.deg.inRad)

    val obeliskSensePos = Pose(48.0, 115.0, 55.deg.inRad)

    val scorePos = Pose(59.0, 104.5, 146.deg.inRad)

    val spikeMark1PosPre = Pose(41.6, 83.1, 180.deg.inRad)

    val spikeMark1PosMiddle = Pose(31.6, 83.1, 180.deg.inRad)

    val spikeMark1PosOuter = Pose(26.6, 83.1, 180.deg.inRad)

    val spikeMark2PosPre = Pose(40.6, 59.0, 180.deg.inRad)

    val spikeMark2PosMiddle = Pose(31.6, 59.0, 180.deg.inRad)

    val spikeMark2PosOuter = Pose(26.6, 59.0, 180.deg.inRad)

    val firstDumpPos = Pose(14.5, 77.5, 180.deg.inRad)

    val secondDumpPos = Pose(14.5, 67.0, 180.deg.inRad)

    val spikeMark3PosPre = Pose(40.6, 35.6, 180.deg.inRad)

    val spikeMark3PosMiddle = Pose(30.6, 35.6, 180.deg.inRad)

    val spikeMark3PosOuter = Pose(25.6, 35.6, 180.deg.inRad)

    val humanPlayerPosPre = Pose(11.8, 19.0, -140.deg.inRad)

    val humanPlayerPosMiddle = Pose(9.0, 9.0, 270.deg.inRad)

    val humanPlayerPosPost = Pose(11.0, 11.4, -140.deg.inRad)

    val outOfTheWayPos = Pose(41.0, 70.0, 180.deg.inRad)

    //endregion

    //region Paths

    lateinit var goalStartToObelisk: PathChain

    lateinit var obeliskToScore: PathChain

    lateinit var scoreToSpikeMark1: PathChain

    lateinit var spikeMark1Pickup: PathChain

    lateinit var spikeMark1PickupToScore: PathChain

    lateinit var scoreToSpikeMark2: PathChain
    lateinit var spikeMark2Pickup: PathChain

    lateinit var spikeMark1ToDump: PathChain
    lateinit var spikeMark2ToDump: PathChain

    lateinit var dumpToScore: PathChain

    lateinit var spikeMark2PickupToScore: PathChain

    lateinit var scoreToSpikeMark3: PathChain
    lateinit var spikeMark3Pickup: PathChain

    lateinit var spikeMark3PickupToScore: PathChain

    lateinit var scoreToOutOfTheWay: PathChain

    lateinit var scoreToGoalZonePark: PathChain


    lateinit var farStartToScore: PathChain

    lateinit var farScoreToSpikeMark3: PathChain

    lateinit var spikeMark3ToFarScore: PathChain

    lateinit var farScoreToHumanPlayer: PathChain

    lateinit var humanPlayerPickup: PathChain

    lateinit var humanPlayerToFarScore: PathChain

    lateinit var farScoreToPark: PathChain


    fun buildTrajectories(follower: Follower) {

        if (AutonomousInfo.redAuto) {
            goalStartToObelisk = follower.pathBuilder()
                .addPath(BezierLine(goalStartPos.mirror(), obeliskSensePos.mirror()))
                .setLinearHeadingInterpolation(goalStartPos.mirror().heading, obeliskSensePos.mirror().heading)
                .build()

            obeliskToScore = follower.pathBuilder()
                .addPath(BezierLine(obeliskSensePos.mirror(), scorePos.mirror()))
                .setLinearHeadingInterpolation(obeliskSensePos.mirror().heading, scorePos.mirror().heading)
                .build()

            scoreToSpikeMark1 = follower.pathBuilder()
                .addPath(
                    BezierCurve(
                        Pose(scorePos.mirror().x, scorePos.mirror().y, scorePos.mirror().heading),
                        Pose(spikeMark1PosPre.x + 30, spikeMark1PosPre.y, spikeMark1PosPre.heading).mirror(),
                        Pose(spikeMark1PosPre.x - redPickupOffset, spikeMark1PosPre.y, spikeMark1PosPre.heading).mirror()
                    )
                )
                .setLinearHeadingInterpolation(scorePos.mirror().heading, spikeMark1PosPre.mirror().heading)
                .build()

            spikeMark1Pickup = follower.pathBuilder()
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


            spikeMark2Pickup = follower.pathBuilder()
                .addPath(BezierLine(Pose(spikeMark2PosMiddle.x - redPickupOffset, spikeMark2PosMiddle.y, spikeMark2PosMiddle.heading).mirror(),
                    Pose(spikeMark2PosOuter.x - redPickupOffset, spikeMark2PosOuter.y, spikeMark2PosOuter.heading).mirror()))
                .setLinearHeadingInterpolation(
                    spikeMark2PosMiddle.mirror().heading,
                    spikeMark2PosOuter.mirror().heading
                )
                .build()

            spikeMark1ToDump = follower.pathBuilder()
                .addPath(BezierCurve(Pose(spikeMark1PosOuter.x - redPickupOffset, spikeMark1PosOuter.y, spikeMark1PosOuter.heading).mirror(),
                    Pose(28.5, firstDumpPos.y).mirror(),
                    firstDumpPos.mirror()))
                .setLinearHeadingInterpolation(spikeMark1PosOuter.mirror().heading, firstDumpPos.mirror().heading)
                .build()

            spikeMark2ToDump = follower.pathBuilder()
                .addPath(BezierCurve(Pose(spikeMark2PosOuter.x - redPickupOffset, spikeMark2PosOuter.y, spikeMark2PosOuter.heading).mirror(),
                    Pose(35.0, secondDumpPos.y).mirror(),
                    secondDumpPos.mirror()))
                .setLinearHeadingInterpolation(spikeMark2PosOuter.mirror().heading, secondDumpPos.mirror().heading)
                .build()

            dumpToScore = follower.pathBuilder()
                .addPath(BezierCurve(Pose(25.0, 71.0, scorePos.heading).mirror(),
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

            spikeMark3Pickup = follower.pathBuilder()
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

            scoreToGoalZonePark = follower.pathBuilder()
                .addPath(BezierLine(scorePos.mirror(), goalZoneParkPos.mirror()))
                .setLinearHeadingInterpolation(scorePos.mirror().heading, goalZoneParkPos.mirror().heading)
                .build()

            farStartToScore = follower.pathBuilder()
                .addPath(BezierLine(farStartPos.mirror(), farShootPos.mirror()))
                .setLinearHeadingInterpolation(farStartPos.mirror().heading, farShootPos.mirror().heading)
                .build()

            farScoreToSpikeMark3 = follower.pathBuilder()
                .addPath(BezierLine(farShootPos.mirror(), spikeMark3PosPre.mirror()))
                .setLinearHeadingInterpolation(farShootPos.mirror().heading, spikeMark3PosPre.mirror().heading)
                .build()

            spikeMark3Pickup = follower.pathBuilder()
                .addPath(BezierLine(spikeMark3PosMiddle.mirror(), spikeMark3PosOuter.mirror()))
                .setLinearHeadingInterpolation(spikeMark3PosMiddle.mirror().heading, spikeMark3PosOuter.mirror().heading)
                .build()

            spikeMark3ToFarScore = follower.pathBuilder()
                .addPath(BezierLine(spikeMark3PosOuter.mirror(), farShootPos.mirror()))
                .setLinearHeadingInterpolation(spikeMark3PosOuter.mirror().heading, farShootPos.mirror().heading)
                .build()

            farScoreToHumanPlayer = follower.pathBuilder()
                .addPath(BezierLine(farShootPos.mirror(), humanPlayerPosPre.mirror()))
                .setLinearHeadingInterpolation(farShootPos.mirror().heading, humanPlayerPosPre.mirror().heading)
                .build()

            humanPlayerPickup = follower.pathBuilder()
                .addPath(BezierLine(humanPlayerPosMiddle.mirror(), humanPlayerPosPost.mirror()))
                .setLinearHeadingInterpolation(humanPlayerPosMiddle.mirror().heading, humanPlayerPosPost.mirror().heading)
                .build()

            humanPlayerToFarScore = follower.pathBuilder()
                .addPath(BezierLine(humanPlayerPosPost.mirror(), farShootPos.mirror()))
                .setLinearHeadingInterpolation(humanPlayerPosPost.mirror().heading, farShootPos.mirror().heading)
                .build()

            farScoreToPark = follower.pathBuilder()
                .addPath(BezierLine(farShootPos.mirror(), farParkPos.mirror()))
                .setLinearHeadingInterpolation(farShootPos.mirror().heading, farParkPos.mirror().heading)
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

            spikeMark1Pickup = follower.pathBuilder()
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

            spikeMark2Pickup = follower.pathBuilder()
                .addPath(BezierLine(Pose(spikeMark2PosMiddle.x - bluePickupOffset, spikeMark2PosMiddle.y, spikeMark2PosMiddle.heading),
                    Pose(spikeMark2PosOuter.x - bluePickupOffset, spikeMark2PosOuter.y, spikeMark2PosOuter.heading)))
                .setLinearHeadingInterpolation(
                    spikeMark2PosMiddle.heading,
                    spikeMark2PosOuter.heading
                )
                .build()


            spikeMark1ToDump = follower.pathBuilder()
                .addPath(BezierCurve(Pose(spikeMark1PosOuter.x - redPickupOffset, spikeMark1PosOuter.y, spikeMark1PosOuter.heading),
                    Pose(28.5, firstDumpPos.y),
                    firstDumpPos))
                .setLinearHeadingInterpolation(spikeMark1PosOuter.heading, firstDumpPos.heading)
                .build()


            spikeMark2ToDump = follower.pathBuilder()
                .addPath(BezierCurve(Pose(spikeMark2PosOuter.x - bluePickupOffset, spikeMark2PosOuter.y, spikeMark2PosOuter.heading),
                    Pose(35.0, secondDumpPos.y),
                    secondDumpPos))
                .setLinearHeadingInterpolation(spikeMark2PosOuter.heading, secondDumpPos.heading)
                .build()

            dumpToScore = follower.pathBuilder()
                .addPath(BezierCurve(firstDumpPos,
                    Pose(25.0, 71.0, scorePos.heading),
                    scorePos))
                .setLinearHeadingInterpolation(spikeMark2PosOuter.heading, scorePos.heading)
                .build()

            spikeMark2PickupToScore = follower.pathBuilder()
                .addPath(BezierCurve(spikeMark2PosOuter,
                    Pose(33.0, 71.0, scorePos.heading),
                    scorePos))
                .setLinearHeadingInterpolation(spikeMark2PosOuter.heading, scorePos.heading)
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

            spikeMark3Pickup = follower.pathBuilder()
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

            scoreToGoalZonePark = follower.pathBuilder()
                .addPath(BezierLine(scorePos, goalZoneParkPos))
                .setLinearHeadingInterpolation(scorePos.heading, goalZoneParkPos.heading)
                .build()

            farStartToScore = follower.pathBuilder()
                .addPath(BezierLine(farStartPos, farShootPos))
                .setLinearHeadingInterpolation(farStartPos.heading, farShootPos.heading)
                .build()

            farScoreToSpikeMark3 = follower.pathBuilder()
                .addPath(BezierLine(farShootPos, Pose(spikeMark3PosPre.x - bluePickupOffset, spikeMark3PosPre.y, spikeMark3PosPre.heading)))
                .setLinearHeadingInterpolation(farShootPos.heading, spikeMark3PosPre.heading)
                .build()

            spikeMark3Pickup = follower.pathBuilder()
                .addPath(BezierLine(spikeMark3PosMiddle, spikeMark3PosOuter))
                .setLinearHeadingInterpolation(spikeMark3PosMiddle.heading, spikeMark3PosOuter.heading)
                .build()

            spikeMark3ToFarScore = follower.pathBuilder()
                .addPath(BezierLine(spikeMark3PosOuter, farShootPos))
                .setLinearHeadingInterpolation(spikeMark3PosOuter.heading, farShootPos.heading)
                .build()

            farScoreToHumanPlayer = follower.pathBuilder()
                .addPath(BezierLine(farShootPos, humanPlayerPosPre))
                .setLinearHeadingInterpolation(farShootPos.heading, humanPlayerPosPre.heading)
                .build()

            humanPlayerPickup = follower.pathBuilder()
                .addPath(BezierLine(humanPlayerPosMiddle, humanPlayerPosPost))
                .setLinearHeadingInterpolation(humanPlayerPosMiddle.heading, humanPlayerPosPost.heading)
                .build()

            humanPlayerToFarScore = follower.pathBuilder()
                .addPath(BezierLine(humanPlayerPosPost, farShootPos))
                .setLinearHeadingInterpolation(humanPlayerPosPost.heading, farShootPos.heading)
                .build()

            farScoreToPark = follower.pathBuilder()
                .addPath(BezierLine(farShootPos, farParkPos))
                .setLinearHeadingInterpolation(farShootPos.heading, farParkPos.heading)
                .build()
        }
    }


    //endregion
}
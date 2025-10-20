package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathBuilder
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.pedroPathing.Constants


object TrajectoryFactory {

    //region Poses

    val goalStartPos = Pose(20.3, 122.8, -36.0)

    val farStartPos = Pose(87.0, 9.0, -90.0)

    val obeliskSensePos = Pose(40.0, 110.0, -135.0)

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


    var builder: PathBuilder = PathBuilder(Constants.createFollower(ActiveOpMode.hardwareMap))

    val goalStartToObelisk = builder
        .addPath(BezierLine(goalStartPos, obeliskSensePos)).build()

    val obeliskToScore = builder
        .addPath(BezierLine(obeliskSensePos, scorePos)).build()

    val scoreToSpikeMark1 = builder
        .addPath(BezierLine(scorePos, spikeMark1PosPre)).build()

    val spikeMark1Pickup = builder
        .addPath(BezierLine(spikeMark1PosPre, spikeMark1PosInner))
        .addPath(BezierLine(spikeMark1PosInner, spikeMark1PosMiddle))
        .addPath(BezierLine(spikeMark1PosMiddle, spikeMark1PosOuter)).build()

    val spikeMark1PickupToScore = builder
        .addPath(BezierLine(spikeMark1PosOuter, scorePos)).build()

    val scoreToSpikeMark2 = builder
        .addPath(BezierLine(scorePos, spikeMark2PosPre)).build()

    val spikeMark2Pickup = builder
        .addPath(BezierLine(spikeMark2PosPre, spikeMark2PosInner))
        .addPath(BezierLine(spikeMark2PosInner, spikeMark2PosMiddle))
        .addPath(BezierLine(spikeMark2PosMiddle, spikeMark2PosOuter)).build()

    val spikeMark2PickupToScore = builder
        .addPath(BezierLine(spikeMark2PosOuter, scorePos)).build()

    val scoreToSpikeMark3 = builder
        .addPath(BezierLine(scorePos, spikeMark3PosPre)).build()

    val spikeMark3Pickup = builder
        .addPath(BezierLine(spikeMark3PosPre, spikeMark3PosInner))
        .addPath(BezierLine(spikeMark3PosInner, spikeMark3PosMiddle))
        .addPath(BezierLine(spikeMark3PosMiddle, spikeMark3PosOuter)).build()

    val spikeMark3PickupToScore = builder
        .addPath(BezierLine(spikeMark3PosOuter, scorePos)).build()



    //endregion
}
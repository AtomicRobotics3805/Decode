package org.firstinspires.ftc.teamcode.autos

import org.firstinspires.ftc.teamcode.subsystems.SpindexerSensor
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.control.KineticState
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.AutoRoutines
import org.firstinspires.ftc.teamcode.Drawing
import org.firstinspires.ftc.teamcode.TrajectoryFactory
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.LimeLight
import org.firstinspires.ftc.teamcode.subsystems.PusherArm
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.subsystems.ZeroSensor


@Autonomous
class ThreeArtifactGoalStart : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(Spindexer, Intake, Shooter, PusherArm, SpindexerSensor, ZeroSensor, LimeLight),
            BulkReadComponent,
        )
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
    }

    lateinit var follower: Follower

    override fun onInit() {
        follower = Constants.createFollower(this.hardwareMap)
        PusherArm.down()
        TrajectoryFactory.buildTrajectories(follower)
        follower.setStartingPose(TrajectoryFactory.goalStartPos)
//        PedroComponent.follower.setStartingPose(TrajectoryFactory.goalStartPos)
        Drawing.init()
    }

    override fun onStartButtonPressed() {
        follower.followPath(TrajectoryFactory.goalStartToObelisk)
//        AutoRoutines.threeArtifactGoalStartAutoRoutine()
    }

    override fun onUpdate() {
        Drawing.drawDebug(follower)
        ActiveOpMode.telemetry.addData("Current pos", follower.pose)
        follower.update()
    }

    override fun onStop() {
        Spindexer.controller.goal = KineticState()
        Shooter.controller.goal = KineticState()
    }
}
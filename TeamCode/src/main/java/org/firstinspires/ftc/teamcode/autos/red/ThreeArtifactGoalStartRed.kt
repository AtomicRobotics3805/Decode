package org.firstinspires.ftc.teamcode.autos.red

import org.firstinspires.ftc.teamcode.subsystems.SpindexerSensor
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
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
import org.firstinspires.ftc.teamcode.autos.AutonomousInfo
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.LimeLight
import org.firstinspires.ftc.teamcode.subsystems.PusherArm
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer


@Autonomous
class ThreeArtifactGoalStartRed : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(Spindexer, Intake, Shooter, PusherArm, SpindexerSensor, LimeLight),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        AutonomousInfo.redAuto = false
    }

//    lateinit var follower: Follower

    override fun onInit() {
        PusherArm.down()
        TrajectoryFactory.buildTrajectories(PedroComponent.follower)
        PedroComponent.follower.setStartingPose(TrajectoryFactory.goalStartPos.mirror())
        Drawing.init()
    }

    override fun onStartButtonPressed() {
        AutoRoutines.threeArtifactGoalStartAutoRoutine()
    }

    override fun onUpdate() {
        Drawing.drawDebug(PedroComponent.follower)
        ActiveOpMode.telemetry.addData("Current pos", PedroComponent.follower.pose)
        ActiveOpMode.telemetry.addData("IsBusy", PedroComponent.follower.isBusy)
//        follower.update()
    }

    override fun onStop() {
        Spindexer.controller.goal = KineticState()
        Shooter.controller.goal = KineticState()
    }
}
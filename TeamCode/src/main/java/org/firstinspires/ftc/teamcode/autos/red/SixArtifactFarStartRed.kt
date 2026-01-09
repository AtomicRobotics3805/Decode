package org.firstinspires.ftc.teamcode.autos.red

import org.firstinspires.ftc.teamcode.subsystems.SpindexerSensor
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.util.RobotLog
import dev.nextftc.control.KineticState
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
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

@Autonomous(name = "\uD83D\uDFE5 Far Start SIX Red", group = "SIX", preselectTeleOp = "Competition TeleOp")
class SixArtifactFarStartRed : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(Spindexer, Intake, Shooter, PusherArm, SpindexerSensor, LimeLight),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
        telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        AutonomousInfo.redAuto = true
    }

//    lateinit var follower: Follower

    override fun onInit() {
        LimeLight.autoRelocalize = false
        LimeLight.obeliskMode = true
        LimeLight.checkPipeline()
        PusherArm.down()
        TrajectoryFactory.buildTrajectories(PedroComponent.follower)
        PedroComponent.follower.setStartingPose(TrajectoryFactory.farStartPos.mirror())
        Drawing.init()
        Spindexer.slots = arrayOf(Spindexer.SpindexerSlotStatus.GREEN, Spindexer.SpindexerSlotStatus.PURPLE,
            Spindexer.SpindexerSlotStatus.PURPLE)
    }

    override fun onStartButtonPressed() {
        AutoRoutines.sixArtifactFarStartAutoRoutine()
    }

    override fun onUpdate() {
        Drawing.drawDebug(PedroComponent.follower)

        RobotLog.d("Motor Amp: Intake Motor: " + Intake.motor.motor.getCurrent(CurrentUnit.AMPS).toString())
        RobotLog.d("Motor Amp: Spindexer Motor: " + Spindexer.motor.motor.getCurrent(CurrentUnit.AMPS).toString())
        RobotLog.d("Motor Amp: Shooter Motor: " + Shooter.motor.motor.getCurrent(CurrentUnit.AMPS).toString())

        ActiveOpMode.telemetry.addData("Current pos", PedroComponent.follower.pose)
        ActiveOpMode.telemetry.addData("IsBusy", PedroComponent.follower.isBusy)

        AutonomousInfo.finalHeading = PedroComponent.follower.heading
        AutonomousInfo.autoEndPos = PedroComponent.follower.pose
    }

    override fun onStop() {
        Spindexer.controller.goal = KineticState()
        Shooter.controller.goal = KineticState()
    }
}
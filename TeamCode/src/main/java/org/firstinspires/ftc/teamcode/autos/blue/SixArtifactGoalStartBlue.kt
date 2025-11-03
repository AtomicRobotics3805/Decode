package org.firstinspires.ftc.teamcode.autos.blue

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.RobotLog
import dev.nextftc.control.KineticState
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.rad
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
import org.firstinspires.ftc.teamcode.subsystems.LimeLight.matchMotif
import org.firstinspires.ftc.teamcode.subsystems.PusherArm
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.subsystems.Spindexer.ticksToAngle
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSensor

@Autonomous(name = "\uD83D\uDFE6 Goal Start SIX", group = "SIX")
class SixArtifactGoalStartBlue : NextFTCOpMode() {

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
        TrajectoryFactory.buildTrajectories(PedroComponent.Companion.follower)
        PedroComponent.Companion.follower.setStartingPose(TrajectoryFactory.goalStartPos)
        Drawing.init()
    }

    override fun onStartButtonPressed() {
        AutoRoutines.sixArtifactGoalStartAutoRoutine()
    }

    override fun onUpdate() {
        Drawing.drawDebug(PedroComponent.Companion.follower)
        ActiveOpMode.telemetry.addData("Current pos", PedroComponent.Companion.follower.pose)
        ActiveOpMode.telemetry.addData("IsBusy", PedroComponent.Companion.follower.isBusy)

        RobotLog.d("Motor Voltage: Drivetrain: " + PedroComponent.follower.drivetrain.voltage.toString())

        RobotLog.d("Motor Amp: Intake Motor: " + Intake.motor.motor.getCurrent(CurrentUnit.AMPS).toString())
        RobotLog.d("Motor Amp: Spindexer Motor: " + Spindexer.motor.motor.getCurrent(CurrentUnit.AMPS).toString())
        RobotLog.d("Motor Amp: Shooter Motor: " + Shooter.motor.motor.getCurrent(CurrentUnit.AMPS).toString())

        ActiveOpMode.telemetry.addData("Current velocity:", Shooter.motor.state.velocity / Shooter.ticksPerRev * 60.0)
        ActiveOpMode.telemetry.addData("Target velocity:", Shooter.controller.goal.velocity / Shooter.ticksPerRev * 60.0)
        ActiveOpMode.telemetry.addData("Shooter power", Shooter.motor.power)

        ActiveOpMode.telemetry.addData("Motif:", matchMotif.toString())

        ActiveOpMode.telemetry.addData("Raw Encoder", Spindexer.motor.state.position)
        ActiveOpMode.telemetry.addData("Angle", ticksToAngle(Spindexer.motor.state.position).normalized.inDeg)
        ActiveOpMode.telemetry.addData("Spindexer goal", Spindexer.controller.goal.position.rad.inDeg)
        ActiveOpMode.telemetry.addData("Spindexer slots", "0:["+ Spindexer.slots[0]+"], 1:["+ Spindexer.slots[1]+"], 2:["+ Spindexer.slots[2]+"]")

        ActiveOpMode.telemetry.update()
//        follower.update()
    }

    override fun onStop() {
        Spindexer.controller.goal = KineticState()
        Shooter.controller.goal = KineticState()
    }
}
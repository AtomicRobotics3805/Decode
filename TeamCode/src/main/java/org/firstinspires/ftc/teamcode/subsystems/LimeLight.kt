package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.ftc.FTCCoordinates
import com.pedropathing.geometry.PedroCoordinates
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.m
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.Routines

@Configurable
object LimeLight : Subsystem {

    //region Motif
    enum class Motif {
        GPP,
        PGP,
        PPG,
        UNKNOWN
    }

    @JvmField
     var matchMotif = Motif.UNKNOWN
    //endregion

    //region Auto Align

    var lastLength = 0
    var axial = 0.0
    var lateral = 0.0
    var yaw = 0.0

    var yawPIDCoefficients = PIDCoefficients(0.0, 0.0, 0.0)
    var axialPIDCoefficients = PIDCoefficients(0.0, 0.0, 0.0)
    var lateralPIDCoefficients = PIDCoefficients(0.0, 0.0, 0.0)

    var axialOffset = 0.0
    var lateralOffset = 0.0
    var yawOffset = 0.0


    var yawPID = controlSystem {
        posPid(yawPIDCoefficients)
    }
    var axialPID = controlSystem {
        posPid(axialPIDCoefficients)
    }
    var lateralPID = controlSystem {
        posPid(lateralPIDCoefficients)
    }

    //endregion

    lateinit var ll: Limelight3A

    override fun initialize() {
        ll = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "limelight")

        ll.pipelineSwitch(0)

        ll.start()
    }

    fun stop() {
        ll.stop()
    }

    override fun periodic() {
        val latestResult = ll.latestResult
//        PedroComponent.follower.pose =
//            Pose(latestResult.botpose.position.x, latestResult.botpose.position.y, latestResult.botpose.orientation.yaw,
//                FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE)
    }

    var detectMotif = InstantCommand {
        val fiducialResults = ll.getLatestResult().fiducialResults
        if (!fiducialResults.isEmpty()) {
            val snapshot = fiducialResults[0]
            matchMotif = when (snapshot.fiducialId) {
                21 -> Motif.GPP
                22 -> Motif.PGP
                else -> Motif.PPG
            }
        }

        Routines.setMotifSelection()
    }

    fun resetPos() {
        val latestResult = ll.latestResult
        PedroComponent.follower.pose =
            Pose(latestResult.botpose.position.x, latestResult.botpose.position.y, latestResult.botpose.orientation.yaw,
                FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE)
    }
}
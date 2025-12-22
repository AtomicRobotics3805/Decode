package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.CompetitionTeleOp
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

    lateinit var ll: Limelight3A

    var autoRelocalize = false

    const val METER_TO_INCH: Double = 39.3701

    override fun initialize() {
        ll = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "limelight")

        ll.pipelineSwitch(0)

        ll.setPollRateHz(100);

        ll.start()
    }

    fun stop() {
        ll.stop()
    }

    var counter = 0
    override fun periodic() {
        if (autoRelocalize && counter >= 5) {
            updatePos()
            counter = 0
        } else if (autoRelocalize) {
            counter++
        }
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

    fun updatePos() {
//        ll.updateRobotOrientation((PedroComponent.follower.heading.rad + 90.deg).inDeg)
        ll.updateRobotOrientation(CompetitionTeleOp.imu.imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES) + 90)
        val result: LLResult? = ll.getLatestResult()

        if (result != null && result.isValid) {
            if (result.staleness <= 100) {
                val newPos = Pose(
                    result.botpose_MT2.position.y * METER_TO_INCH + 72,
                    (-result.botpose_MT2.position.x * METER_TO_INCH + 72),
                    PedroComponent.follower.heading
                )
                if (newPos.x != 72.0 && newPos.x > 0.0 && newPos.x < 144.0) {
                    if (newPos.y != 72.0 && newPos.y > 0.0 && newPos.y < 144.0) {
                        PedroComponent.follower.pose = newPos
                    }
                }
            }
        }
    }
}
package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.deg
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.CompetitionTeleOp
import org.firstinspires.ftc.teamcode.Routines
import org.firstinspires.ftc.teamcode.autos.AutonomousInfo


@Configurable
object LimeLight : Subsystem {

    var obeliskMode = false

    var TX: Double? = 0.0

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

    var readings = mutableListOf<Pose>()

    const val METER_TO_INCH: Double = 39.3701

    override fun initialize() {
        ll = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "limelight")

        if (obeliskMode) {
            ll.pipelineSwitch(1)
        } else {
            ll.pipelineSwitch(0)
        }

        ll.setPollRateHz(100);

        ll.start()
    }

    fun stop() {
        ll.stop()
    }

    var counter = 0
    override fun periodic() {

    }

    fun getTX(): Double {
        var tX = 3805.0
        if (AutonomousInfo.redAuto) {
            for (tag in ll.latestResult.fiducialResults) {
                if (tag.fiducialId == 24) {
                    tX = tag.targetXDegrees.deg.inRad
                }
            }
        } else {
            for (tag in ll.latestResult.fiducialResults) {
                if (tag.fiducialId == 20) {
                    tX = tag.targetXDegrees.deg.inRad
                }
            }
        }
        return tX
    }

    fun checkPipeline() {
        if (obeliskMode) {
            ll.pipelineSwitch(1)
        } else {
            ll.pipelineSwitch(0)
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
        ll.updateRobotOrientation(CompetitionTeleOp.imu.get().inDeg + 90)
        val result: LLResult? = ll.getLatestResult()
//
//        TX = result?.fiducialResults?.get(0)?.targetXDegrees

        if (result != null && result.isValid) {
            if (result.staleness <= 100) {
                val newPos = if (result.botposeTagCount == 1) {
                    Pose(
                        result.fiducialResults[0].robotPoseFieldSpace.position.y * METER_TO_INCH + 72,
                        (-result.fiducialResults[0].robotPoseFieldSpace.position.x * METER_TO_INCH + 72),
                        PedroComponent.follower.heading
                    )
                } else {
                    Pose(
                        result.botpose_MT2.position.y * METER_TO_INCH + 72,
                        (-result.botpose_MT2.position.x * METER_TO_INCH + 72),
                        PedroComponent.follower.heading
                    )
                }
                if (newPos.x != 72.0 && newPos.x > 0.0 && newPos.x < 144.0) {
                    if (newPos.y != 72.0 && newPos.y > 0.0 && newPos.y < 144.0) {
//
//                        readings.add(newPos) // Add newest reading
//                        if (readings.size > 5) { // Only averages last 5 readings
//                            readings.removeAt(0)    // Remove oldest reading
//                        }
//                        val sumPose = readings.reduce { accumulator, next ->
//                            Pose(accumulator.x + next.x,
//                                accumulator.y + next.y,
//                                accumulator.heading + next.heading)
//                        }
//
//                        PedroComponent.follower.pose = Pose(sumPose.x / readings.size, sumPose.y / readings.size, sumPose.heading / readings.size)
                        PedroComponent.follower.pose = newPos
                    }
                }
            }
        }
    }
}
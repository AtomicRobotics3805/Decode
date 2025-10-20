package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode

object LimeLight : Subsystem {

    //region Motif
    enum class Motif {
        GPP,
        PGP,
        PPG,
        UNKNOWN
    }

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

    var detectMotif = InstantCommand {
        val fiducialResults = ll.getLatestResult().fiducialResults
        if (!fiducialResults.isEmpty()) {
            val snapshot = fiducialResults[0]
            matchMotif = when (snapshot.fiducialId) {
                21 -> Motif.GPP
                22 -> Motif.PGP
                else -> Motif.PPG
            }
            ActiveOpMode.telemetry.addData("Motif:", matchMotif.toString())
        }
    }

    fun autoAlign(alignTag: Int) {
        val fiducialResults = ll.getLatestResult().fiducialResults
        if (!fiducialResults.isEmpty()) {
            val snapshot = fiducialResults[0]

            if (snapshot.fiducialId == alignTag) {
                if (lastLength != fiducialResults.size) {
                    axial = axialPID.calculate(
                        KineticState(
                            axialOffset - snapshot.robotPoseTargetSpace.position.z
                        )
                    )
                    lateral = lateralPID.calculate(
                        KineticState(
                            -lateralOffset + snapshot.robotPoseTargetSpace.position.x
                        )
                    )
                    yaw = yawPID.calculate(
                        KineticState(
                            -yawOffset - snapshot.robotPoseTargetSpace.orientation.yaw
                        )
                    )
                } else {
                    axial = 0.0
                    lateral = 0.0
                    yaw = 0.0
                }
                ActiveOpMode.telemetry.addData("Tag ID:", snapshot.fiducialId)
                ActiveOpMode.telemetry.addData("Axial:", -snapshot.robotPoseTargetSpace.position.z)
                ActiveOpMode.telemetry.addData("Lateral:", snapshot.robotPoseTargetSpace.position.x)
                ActiveOpMode.telemetry.addData("Yaw:",snapshot.robotPoseTargetSpace.orientation.yaw)
            }
        }
        lastLength = fiducialResults.size
    }
}
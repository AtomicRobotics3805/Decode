package org.firstinspires.ftc.teamcode

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.LLResultTypes
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.m
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent
import org.firstinspires.ftc.teamcode.autos.AutonomousInfo
import org.firstinspires.ftc.teamcode.subsystems.LimeLight
import java.util.function.Supplier
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt

object AutoAdjustingCalc {

    val goalPos = Pose(16.3, 131.8)

    var pose = Pose()

    fun calculatePower(): Double {
        val currentPos = PedroComponent.follower.pose
        var distance = 0.0

        distance = if (!LimeLight.ll.latestResult.fiducialResults.any { it.fiducialId == if (AutonomousInfo.redAuto) 24 else 20 } ) {
            if (AutonomousInfo.redAuto) {
                sqrt(((goalPos.mirror().x - currentPos.x) * (goalPos.mirror().x - currentPos.x)) + ((goalPos.mirror().y - currentPos.y) * (goalPos.mirror().y - currentPos.y)))
            } else {
                sqrt(((goalPos.x - currentPos.x) * (goalPos.x - currentPos.x)) + ((goalPos.y - currentPos.y) * (goalPos.y - currentPos.y)))
            }
        } else {
            abs(LimeLight.ll.latestResult.fiducialResults[0].robotPoseTargetSpace.position.z.m.inIn)
        }

        return (0.0937119*(distance*distance)) + (-2.1386 * distance) + 2042.09419
    }

    fun calculateAimAngle(): Double {
        val currentPos = PedroComponent.follower.pose



        if (LimeLight.ll.latestResult.fiducialResults.any { it.fiducialId == if (AutonomousInfo.redAuto) 24 else 20 } ) {
            if (PedroComponent.follower.pose.y <= 48.0) {
                val heading = PedroComponent.follower.heading - LimeLight.getTX() + if (AutonomousInfo.redAuto) { 1.deg.inRad } else { -1.deg.inRad }
                pose = PedroComponent.follower.pose.withHeading(heading)
            } else if (PedroComponent.follower.pose.y >= 130.0) {
                val heading = PedroComponent.follower.heading - LimeLight.getTX() + if (AutonomousInfo.redAuto) { -1.deg.inRad } else { 1.deg.inRad }
                pose = PedroComponent.follower.pose.withHeading(heading)
            } else {
                val heading = PedroComponent.follower.heading - LimeLight.getTX()
                pose = PedroComponent.follower.pose.withHeading(heading)
            }
        }

        if (PedroComponent.follower.pose.distanceFrom(pose) < 10.0) {
            return pose.heading
        } else {
            if (AutonomousInfo.redAuto) {
                val dx = 144 - currentPos.x
                val dy = 144 - currentPos.y

                return atan2(dy, dx)
            } else {
                val dx = 0 - currentPos.x
                val dy = 144 - currentPos.y

                return atan2(dy, dx)
            }
        }
    }

}

class PIDJoystickBlend(val joystick: Supplier<Double>, val pid: Supplier<Double>): Supplier<Double> {
    var usePID = false

    override fun get(): Double {
        return if (usePID) pid.get() else joystick.get()
    }
}
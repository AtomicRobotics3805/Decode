package org.firstinspires.ftc.teamcode

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent
import org.firstinspires.ftc.teamcode.autos.AutonomousInfo
import java.util.function.Supplier
import kotlin.math.atan2
import kotlin.math.sqrt

object AutoAdjustingCalc {

    val goalPos = Pose(16.3, 131.8)

    fun calculatePower(): Double {
        val currentPos = PedroComponent.follower.pose

        var distance = 0.0
        distance = if (AutonomousInfo.redAuto) {
            sqrt(((goalPos.mirror().x - currentPos.x)*(goalPos.mirror().x - currentPos.x)) + ((goalPos.mirror().y - currentPos.y)*(goalPos.mirror().y - currentPos.y)))
        } else {
            sqrt(((goalPos.x - currentPos.x)*(goalPos.x - currentPos.x)) + ((goalPos.y - currentPos.y)*(goalPos.y - currentPos.y)))

        }

        return (0.0303725*(distance*distance)) + (5.20557 * distance) + 1839.13148
    }

    fun calculateAimAngleOld(): Double {
        val currentPos = PedroComponent.follower.pose

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

class PIDJoystickBlend(val joystick: Supplier<Double>, val pid: Supplier<Double>): Supplier<Double> {
    var usePID = false

    override fun get(): Double {
        return if (usePID) pid.get() else joystick.get()
    }
}
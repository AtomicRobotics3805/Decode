package org.firstinspires.ftc.teamcode

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

        if (AutonomousInfo.redAuto) {
            val distance = sqrt(((goalPos.mirror().x - currentPos.x)*(goalPos.mirror().x - currentPos.x)) + ((goalPos.mirror().y - currentPos.y)*(goalPos.mirror().y - currentPos.y)))

            return (0.00559054*(distance*distance)) + (10.59177 * distance) + 1906.02268
        }
        else {
            val distance = sqrt(((goalPos.x - currentPos.x)*(goalPos.x - currentPos.x)) + ((goalPos.y - currentPos.y)*(goalPos.y - currentPos.y)))

            return (0.00559054*(distance*distance)) + (10.59177 * distance) + 1906.02268
        }
    }

    fun calculateAimAngle(): Double {
        val currentPos = PedroComponent.follower.pose

        if (AutonomousInfo.redAuto) {
            val dx = 142 - currentPos.x
            val dy = 142 - currentPos.y

            return atan2(dx, dy)
        } else {
            val dx = 2 - currentPos.x
            val dy = 142 - currentPos.y

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
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

            return (0.0357839*(distance*distance)) + (8.43768 * distance) + 1714.48071
        }
        else {
            val distance = sqrt(((goalPos.x - currentPos.x)*(goalPos.x - currentPos.x)) + ((goalPos.y - currentPos.y)*(goalPos.y - currentPos.y)))

            return (0.0357839*(distance*distance)) + (8.43768 * distance) + 1714.48071
        }
    }

    fun calculateAimAngle(): Double {
        val currentPos = PedroComponent.follower.pose

        if (AutonomousInfo.redAuto) {
            val dx = 141 - currentPos.x
            val dy = 141 - currentPos.y

            return atan2(dy, dx)
        } else {
            val dx = 3 - currentPos.x
            val dy = 141 - currentPos.y

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
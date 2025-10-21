package org.firstinspires.ftc.teamcode

import com.pedropathing.geometry.Pose
import dev.nextftc.core.commands.Command
import dev.nextftc.extensions.pedro.PedroComponent
import java.lang.Math.pow
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Drives robot to a specified point
 * @param point the point to go to
 * @param posTolerance the distance (in inches) away from the specified point in order to consider the robot at the location
 * @param headingTolerance the distance (in radians) away from the specified target in order to consider the robot at the setpoint
 */
class PIDToPoint(val point: Pose, val posTolerance: Double, val headingTolerance: Double) : Command() {
    override val isDone: Boolean
        get() = (
                sqrt(
                    point.x - PedroComponent.follower.pose.x).pow(2) +
                    (point.y - PedroComponent.follower.pose.y).pow(2)) < posTolerance
                && abs(point.heading - PedroComponent.follower.pose.heading) < headingTolerance

    override fun start() {
        PedroComponent.follower.holdPoint(point)
    }
}
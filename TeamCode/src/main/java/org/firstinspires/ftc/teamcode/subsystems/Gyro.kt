/*
 * NextFTC: a user-friendly control library for FIRST Tech Challenge
 *     Copyright (C) 2025 Rowan McAlpin
 *
 *     This program is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.IMU
import dev.nextftc.control.feedback.AngleType
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.delegates.LazyHardware
import org.firstinspires.ftc.teamcode.subsystems.Direction.BACKWARD
import org.firstinspires.ftc.teamcode.subsystems.Direction.DOWN
import org.firstinspires.ftc.teamcode.subsystems.Direction.FORWARD
import org.firstinspires.ftc.teamcode.subsystems.Direction.LEFT
import org.firstinspires.ftc.teamcode.subsystems.Direction.RIGHT
import org.firstinspires.ftc.teamcode.subsystems.Direction.UP
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import java.util.function.Supplier

class Gyro(imuFactory: () -> IMU) : Supplier<Angle> {

    private val lazy = LazyHardware(imuFactory)
    val imu by lazy

    public var offset = 0.rad

    constructor(imu: IMU) : this({ imu })
    constructor(
        name: String,
        logoFacingDirection: Direction,
        usbFacingDirection: Direction
    ) : this({
        val imu = ActiveOpMode.hardwareMap[name] as IMU
        imu.initialize(
            IMU.Parameters(
                RevHubOrientationOnRobot(
                    logoFacingDirection.toLogoDirection(),
                    usbFacingDirection.toUsbDirection()
                )
            )
        )
        imu
    })

    fun zero() = lazy.applyAfterInit {
        offset = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS).rad
    }

    fun zeroed() = apply { zero() }

    override fun get() = Angle.fromRad(
        imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS) - offset.inRad
    )
}

enum class Direction {
    UP,
    DOWN,
    LEFT,
    RIGHT,
    FORWARD,
    BACKWARD;
}

fun Direction.toLogoDirection() = when (this) {
    UP -> RevHubOrientationOnRobot.LogoFacingDirection.UP
    DOWN -> RevHubOrientationOnRobot.LogoFacingDirection.DOWN
    LEFT -> RevHubOrientationOnRobot.LogoFacingDirection.LEFT
    RIGHT -> RevHubOrientationOnRobot.LogoFacingDirection.RIGHT
    FORWARD -> RevHubOrientationOnRobot.LogoFacingDirection.FORWARD
    BACKWARD -> RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD
}

fun Direction.toUsbDirection() = when (this) {
    UP -> RevHubOrientationOnRobot.UsbFacingDirection.UP
    DOWN -> RevHubOrientationOnRobot.UsbFacingDirection.DOWN
    LEFT -> RevHubOrientationOnRobot.UsbFacingDirection.LEFT
    RIGHT -> RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
    FORWARD -> RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
    BACKWARD -> RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
}
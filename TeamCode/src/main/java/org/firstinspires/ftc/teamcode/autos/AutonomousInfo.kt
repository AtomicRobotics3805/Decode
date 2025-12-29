package org.firstinspires.ftc.teamcode.autos

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import dev.nextftc.core.units.deg
import org.firstinspires.ftc.teamcode.subsystems.Spindexer

@Configurable
object AutonomousInfo {

    @JvmField
    var redAuto = false

    var finalHeading = 0.0.deg.inRad

    var autoRunning = false

    var autoEndPos = Pose(72.0, 72.0)

    var autoRan = false

}
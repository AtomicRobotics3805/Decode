package org.firstinspires.ftc.teamcode

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.bindings.button
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.PusherArm
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Spindexer

@TeleOp
class TestOpMode : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(Spindexer, Intake, Shooter),
            BulkReadComponent,
            BindingsComponent
        )
    }

    val dpadLeft = button { FakeInputs.dpadLeft }
    val dpadUp = button { FakeInputs.dpadUp }
    val dpadRight = button { FakeInputs.dpadRight }
    val fakeButton = button { FakeInputs.fakeButton }

    val intake = button { FakeInputs.intake }

    val shooter = button { FakeInputs.shooter }

    override fun onStartButtonPressed() {
        dpadUp.whenBecomesTrue { Spindexer.setAngle(0.rad).invoke() }
        dpadLeft.whenBecomesTrue { Spindexer.setAngle(120.deg).invoke() }
        dpadRight.whenBecomesTrue { Spindexer.setAngle((-120).deg).invoke() }
        fakeButton.whenBecomesTrue { Spindexer.setAngle(FakeInputs.newAngle.deg).schedule() }
        intake.whenBecomesTrue { Intake.start() }
        intake.whenBecomesFalse { Intake.stop() }
        shooter.whenBecomesTrue { Shooter.start() }
        shooter.whenBecomesFalse { Shooter.stop() }
    }

}
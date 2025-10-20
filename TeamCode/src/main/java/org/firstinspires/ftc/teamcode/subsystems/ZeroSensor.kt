package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode

object ZeroSensor : Subsystem {
    lateinit var sensor: DigitalChannel

    override fun initialize() {
        sensor = ActiveOpMode.hardwareMap.get(DigitalChannel::class.java, "zero_sensor")

        sensor.mode = DigitalChannel.Mode.INPUT
    }
    override fun periodic() {
        // TODO: Only check every X loops (probably 3)

        if (sensor.state) {
            Spindexer.motor.encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        }

        ActiveOpMode.telemetry.addData("Magnet", sensor.state)
    }
}
package org.firstinspires.ftc.teamcode

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.utility.InstantCommand

object Callbacks {
    enum class Callback {
        FIRST_PICKUP_COMPLETE,
        SECOND_PICKUP_COMPLETE
    }

    val finishedCallbacks: MutableList<Callback> = mutableListOf()

    fun reset() {
        finishedCallbacks.clear()
    }

    fun setCallback(callback: Callback): Command {
        return InstantCommand { finishedCallbacks.add(callback) }
    }

    fun waitFor(callback: Callback): Command {
        return WaitUntil { finishedCallbacks.contains(callback) }
    }
}
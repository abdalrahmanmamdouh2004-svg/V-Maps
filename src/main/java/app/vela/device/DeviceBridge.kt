package app.vela.device

import kotlinx.coroutines.flow.MutableStateFlow

/**
 * What the phone's map knows that a paired nav device shows, handed across without the device
 * service reaching into the view model. The same idea as [app.vela.car.CarBridge].
 */
object DeviceBridge {
    /** The posted limit of the road under the puck, as the phone shows it. Null when unknown. */
    val speedLimitKmh = MutableStateFlow<Double?>(null)

    /** The watch asked to be taken Home or to Work. Nothing listens while the map's view model is gone (the app
     *  closed): the press is then dropped, and the watch tells you to open Vela. */
    val watchGo = kotlinx.coroutines.flow.MutableSharedFlow<app.vela.core.model.ShortcutKind>(extraBufferCapacity = 4)
}

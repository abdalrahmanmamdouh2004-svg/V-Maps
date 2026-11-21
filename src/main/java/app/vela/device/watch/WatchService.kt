package app.vela.device.watch

import android.Manifest
import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.bluetooth.BluetoothManager
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.content.pm.ServiceInfo
import android.location.Location
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Build
import android.os.IBinder
import android.os.PowerManager
import android.os.SystemClock
import android.util.Log
import androidx.compose.runtime.mutableStateOf
import androidx.core.app.NotificationCompat
import androidx.core.content.ContextCompat
import app.vela.MainActivity
import app.vela.R
import app.vela.core.data.naming.RoadNameTiles
import app.vela.core.location.LocationProvider
import app.vela.core.model.LatLng
import app.vela.core.model.Route
import app.vela.core.model.distanceTo
import app.vela.core.nav.NavSession
import app.vela.core.nav.RouteProjection
import app.vela.device.DeviceBridge
import app.vela.device.DeviceRoads
import app.vela.device.DeviceNavFrame
import app.vela.device.DeviceService
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import kotlin.collections.ArrayDeque
import javax.inject.Inject
import kotlin.math.roundToInt

/**
 * Shows the drive on a CMF Watch Pro 2 running the Vela Nav watch app. It holds a second GATT
 * client on the link Nothing X already has ([WatchSession]), and while the watch app is open it
 * sends the drive once a second: the turn, the road, what is left, and the route line with the
 * car on it (protocol `n1`, the watch app's `src/proto.rs`). The watch asks for everything again
 * (`n1Q`) when it opens or loses a message; the answer ends with a state flagged as a snapshot.
 *
 * Like [DeviceService] it only reads [NavSession.state] and [LocationProvider.updates].
 */
@AndroidEntryPoint
class WatchService : Service() {

    @Inject lateinit var navSession: NavSession
    @Inject lateinit var locationProvider: LocationProvider

    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.Main.immediate)
    private var loop: Job? = null
    private var locationJob: Job? = null
    private var foreground = false
    private var wake: PowerManager.WakeLock? = null

    private var lastFix: Location? = null
    private var course = 0f
    /** Smoothed azimuth of the phone's top edge, used for pedestrian navigation. */
    private var phoneHeadingDeg: Float? = null
    private var lastHeadingNs = 0L
    private val sensorManager by lazy { getSystemService(Context.SENSOR_SERVICE) as SensorManager }
    private val rotationMatrix = FloatArray(9)
    private val orientationAngles = FloatArray(3)
    private val headingListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            if (event.sensor.type != Sensor.TYPE_ROTATION_VECTOR) return
            SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values)
            SensorManager.getOrientation(rotationMatrix, orientationAngles)
            val raw = ((Math.toDegrees(orientationAngles[0].toDouble()).toFloat() % 360f) + 360f) % 360f
            val previous = phoneHeadingDeg
            // smoothed over time, not over samples: while walking, a phone in a pocket swings with the leg
            // (about once a second), and a long average takes that out
            val dt = ((event.timestamp - lastHeadingNs).coerceIn(0L, 200_000_000L)) / 1e9f
            lastHeadingNs = event.timestamp
            val walking = lastFix?.let { it.hasSpeed() && it.speed >= WALK_SMOOTH_MPS } == true
            val tau = if (walking) HEADING_TAU_WALKING_S else HEADING_TAU_STILL_S
            val filtered = if (previous == null) raw else {
                val delta = ((raw - previous + 540f) % 360f) - 180f
                (previous + delta * (dt / (tau + dt)) + 360f) % 360f
            }
            phoneHeadingDeg = filtered
            val fix = lastFix
            // While walking or standing, the phone compass is more useful than GPS course.
            if (fix == null || !fix.hasSpeed() || fix.speed <= WALKING_HEADING_MAX_MPS) course = filtered
        }
        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit
    }
    private var cumFor: Route? = null
    private var cum = DoubleArray(0)

    // what the open watch app holds
    private var seq = 0
    private var routeId = 0
    private var sentRoute: Route? = null
    private var geometry: WatchDrive.Geometry? = null
    private var sentText: String? = null
    private var wantSnapshot = true
    /** Centre of the real nearby-road set currently held by the watch. */
    private var roadsCenter: LatLng? = null
    private var roadsRetryAt = 0L
    private var roadsId = 0
    /** The streets are fetched in the background and sent a few messages per tick, after the state, so the
     *  car never waits for a tile or for fifteen round trips. */
    private class Fetched(val here: LatLng, val origin: LatLng, val lines: List<List<IntArray>>?)
    private var roadsJob: Job? = null
    @Volatile private var roadsFetched: Fetched? = null
    private val roadsQueue = ArrayDeque<Pair<Char, ByteArray>>()
    private var roadsStale = false

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        WatchPrefs.load(this)
        if (intent?.action == ACTION_STOP || !WatchPrefs.enabled.value || WatchPrefs.address.value == null ||
            !DeviceService.hasBluetooth(this)
        ) {
            shutDown()
            return START_NOT_STICKY
        }
        if (!foreground) {
            try {
                startForegroundTyped(notification(state.value))
                foreground = true
            } catch (t: Throwable) {
                Log.w(TAG, "foreground start refused: ${t.javaClass.simpleName}")
                shutDown()
                return START_NOT_STICKY
            }
        }
        startLocation()
        startHeading()
        if (wake?.isHeld != true) {
            wake = getSystemService(PowerManager::class.java)
                ?.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "vela:watch")
                ?.apply { setReferenceCounted(false); runCatching { acquire(8 * 60 * 60 * 1000L) } }
        }
        if (loop?.isActive != true) loop = scope.launch { run() }
        return START_STICKY
    }

    override fun onDestroy() {
        runCatching { wake?.takeIf { it.isHeld }?.release() }
        stopHeading()
        scope.cancel()
        publish(State.OFF)
        super.onDestroy()
    }

    private fun startHeading() {
        val sensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) ?: return
        sensorManager.registerListener(headingListener, sensor, SensorManager.SENSOR_DELAY_GAME)
    }
    private fun stopHeading() {
        runCatching { sensorManager.unregisterListener(headingListener) }
    }
    private fun startLocation() {
        if (locationJob != null) return
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) !=
            PackageManager.PERMISSION_GRANTED
        ) return
        locationJob = scope.launch {
            locationProvider.updates().collect { loc ->
                if (loc.latitude == 0.0 && loc.longitude == 0.0) return@collect
                lastFix = loc
                course = courseFor(loc)
            }
        }
    }

    /** Connect, feed, and on any loss connect again, backing off to half a minute. */
    private suspend fun run() {
        var backoff = 2_000L
        while (scope.isActive) {
            val device = WatchPrefs.address.value?.let { a ->
                getSystemService(BluetoothManager::class.java)?.adapter?.takeIf { it.isEnabled }
                    ?.bondedDevices?.firstOrNull { it.address == a }
            }
            if (device == null) {
                publish(State.CONNECTING)
                delay(backoff)
                backoff = (backoff * 2).coerceAtMost(30_000L)
                continue
            }
            publish(State.CONNECTING)
            val session = WatchSession(this)
            try {
                session.open(device)
                backoff = 2_000L
                feed(session)
            } catch (t: Throwable) {
                if (t is kotlinx.coroutines.CancellationException) throw t
                Log.w(TAG, "watch link: ${t.javaClass.simpleName}")
            } finally {
                session.close()
            }
            delay(backoff)
            backoff = (backoff * 2).coerceAtMost(30_000L)
        }
    }

    /** Runs while the link is up. Returns when it is lost. */
    private suspend fun feed(session: WatchSession) {
        var appOpen = false
        var silent = 0
        forgetWatch()
        while (session.isOpen) {
            if (!appOpen) {
                val pong = session.ping() ?: if (++silent >= 3) return else null
                if (pong != null) silent = 0
                appOpen = pong?.appOpen == true
                publish(if (appOpen) State.SHOWING else State.CONNECTED)
                if (!appOpen) {
                    forgetWatch()
                    delay(IDLE_PING_MS)
                    continue
                }
            }
            while (true) {
                val e = session.events.tryReceive().getOrNull() ?: break
                when {
                    e.contentEquals(WatchNav.ASK) -> wantSnapshot = true
                    e.contentEquals(WatchNav.HOME) -> DeviceBridge.watchGo.tryEmit(app.vela.core.model.ShortcutKind.HOME)
                    e.contentEquals(WatchNav.WORK) -> DeviceBridge.watchGo.tryEmit(app.vela.core.model.ShortcutKind.WORK)
                }
            }
            when (val r = push(session)) {
                WatchSession.OK -> silent = 0
                WatchSession.NO_APP -> appOpen = false
                WatchSession.NO_REPLY -> if (++silent >= 3) return
                else -> Log.w(TAG, "watch refused a message: $r")
            }
            delay(TICK_MS)
        }
    }

    /** The watch app closed (or never opened): it holds nothing, so the next open starts over. */
    private fun forgetWatch() {
        sentRoute = null
        sentText = null
        roadsCenter = null
        roadsRetryAt = 0L
        roadsJob?.cancel()
        roadsJob = null
        roadsFetched = null
        roadsQueue.clear()
        roadsStale = false
        wantSnapshot = true
    }

    private suspend fun send(session: WatchSession, build: (Int) -> ByteArray): Int {
        val r = session.send(build(seq and 0xFF))
        if (r == WatchSession.OK) seq++ // the watch counts only what reached it
        return r
    }

    /** One tick: a snapshot when asked, else what changed and the state. */
    private suspend fun push(session: WatchSession): Int {
        val s = navSession.state.value
        val frame = DeviceNavFrame.of(s)
        val snapshot = wantSnapshot
        val text = WatchDrive.line(frame, s.destinationLabel)
        if (snapshot || text != sentText) {
            val r = send(session) { WatchNav.text(it, text) }
            if (r != WatchSession.OK) return r
            sentText = text
        }
        val route = s.route?.takeIf { frame.active }
        val routeChanged = snapshot || route !== sentRoute
        if (routeChanged) {
            val g = route?.let { WatchDrive.geometry(it.polyline) }
            if (g != null) {
                routeId = (routeId + 1) and 0xFF
                for ((kind, body) in WatchNav.routeParts(routeId, g.points)) {
                    val r = send(session) { WatchNav.msg(kind, it, body) }
                    if (r != WatchSession.OK) return r
                }
            }
            geometry = g
            sentRoute = route
        }

        // Send the real nearby street network separately from the selected blue route. Reuse
        // Vela's vector tiles and refresh after 250 m of travel, or whenever the watch asks for
        // a complete snapshot / the active route changes.
        val here = lastFix?.let { LatLng(it.latitude, it.longitude) }
        val now = android.os.SystemClock.elapsedRealtime()
        if (routeChanged) roadsStale = true
        if (route != null && geometry != null && here != null) {
            val previous = roadsCenter
            val roadsDue = roadsStale || (previous == null && now >= roadsRetryAt) ||
                (previous != null && previous.distanceTo(here) >= ROADS_REFRESH_M)
            if (roadsDue && roadsJob?.isActive != true) {
                roadsStale = false
                val origin = geometry!!.origin
                roadsJob = scope.launch {
                    val lines = withContext(Dispatchers.Default) {
                        RoadNameTiles.roadsAlong(DeviceRoads.probe(here))?.let { WatchRoads.geometry(here, it) }
                    }
                    roadsFetched = Fetched(here, origin, lines)
                }
            }
        } else if (route != null && geometry != null && (snapshot || routeChanged)) {
            // A route can be active before a location fix is available; don't leave stale streets.
            roadsId = (roadsId + 1) and 0xFF
            for ((kind, body) in WatchNav.roadsParts(roadsId, intArrayOf(0, 0), emptyList())) {
                val sent = send(session) { WatchNav.msg(kind, it, body) }
                if (sent != WatchSession.OK) return sent
            }
            roadsCenter = null
            roadsRetryAt = 0L
        } else if (route == null) {
            roadsCenter = null
        }
        roadsFetched?.let { f ->
            roadsFetched = null
            if (route != null && geometry?.origin == f.origin) {
                roadsId = (roadsId + 1) and 0xFF
                roadsQueue.clear()
                roadsQueue.addAll(WatchNav.roadsParts(roadsId, WatchDrive.metres(f.origin, f.here), f.lines ?: emptyList()))
                if (f.lines != null) {
                    roadsCenter = f.here
                    roadsRetryAt = 0L
                } else {
                    roadsCenter = null
                    roadsRetryAt = now + ROADS_RETRY_MS
                }
            }
        }
        val r = send(session) { WatchNav.state(it, stateFor(s, frame, snapshot)) }
        if (r != WatchSession.OK) return r
        if (snapshot) wantSnapshot = false
        var sentNow = 0
        while (sentNow < ROADS_PER_TICK && roadsQueue.isNotEmpty()) {
            val (kind, body) = roadsQueue.first()
            val sent = send(session) { WatchNav.msg(kind, it, body) }
            if (sent != WatchSession.OK) return sent
            roadsQueue.removeFirst()
            sentNow++
        }
        return r
    }

    private fun stateFor(s: NavSession.State, frame: DeviceNavFrame, snapshot: Boolean): WatchNav.State {
        val g = geometry
        val fix = lastFix
        val imperial = app.vela.ui.Units.imperial.value
        val kmh = DeviceBridge.speedLimitKmh.value
        val limit = if (kmh == null) 0 else (if (imperial) kmh / 1.609344 else kmh).roundToInt()
        val total = s.route?.distanceMeters ?: 0.0
        val fraction = if (total > 0) (total - s.remainingDistance) / total else 0.0
        return WatchDrive.state(
            frame = frame,
            arrived = s.arrived,
            imperial = imperial,
            clock24 = app.vela.ui.Clock24.on.value,
            fix = fix != null && fix.hasAccuracy() && fix.accuracy < FIX_ACCURACY_M,
            limit = limit,
            position = if (g != null && fix != null) {
                WatchDrive.metres(g.origin, deadReckoned(fix))
            } else null,
            courseDeg = course,
            progress = if (g != null) WatchDrive.progress(g, fraction) else 0,
            snapshot = snapshot,
        )
    }

    /**
     * The fix moved on along [course] at its speed for the time since it was taken (at most 2.5 s). A GPS fix
     * comes about once a second; this way every update to the watch has the car a little further on.
     */
    private fun deadReckoned(fix: Location): app.vela.core.model.LatLng {
        val ageS = ((SystemClock.elapsedRealtimeNanos() - fix.elapsedRealtimeNanos) / 1e9).coerceIn(0.0, 2.5)
        val v = if (fix.hasSpeed() && fix.speed > MOVING_MPS) fix.speed.toDouble() else 0.0
        val d = v * ageS
        val rad = Math.toRadians(course.toDouble())
        val dLat = d * Math.cos(rad) / 111_320.0
        val dLon = d * Math.sin(rad) / (111_320.0 * Math.cos(Math.toRadians(fix.latitude)))
        return app.vela.core.model.LatLng(fix.latitude + dLat, fix.longitude + dLon)
    }

    /** As [DeviceService]: the road's own direction on a drive, else the GPS course while moving. */
    private fun courseFor(loc: Location): Float {
        val s = navSession.state.value
        val route = s.route
        if (route != null && s.navigating && !s.paused && !s.nav.offRoute && route.polyline.size >= 2) {
            if (cumFor !== route) {
                cumFor = route
                cum = RouteProjection.cumulative(route.polyline)
            }
            val along = (cum.last() - s.remainingDistance).coerceIn(0.0, cum.last())
            val routeBearing = RouteProjection.bearingAt(route.polyline, cum, along).toFloat()
            // At walking pace (or stopped at a crossing), follow the phone's live compass heading.
            // At vehicle speed, keep the stable route/GPS direction instead of reacting to phone handling.
            if (!loc.hasSpeed() || loc.speed <= WALKING_HEADING_MAX_MPS) return phoneHeadingDeg ?: routeBearing
            return routeBearing
        }
        return if (loc.hasBearing() && loc.hasSpeed() && loc.speed > MOVING_MPS) loc.bearing
            else phoneHeadingDeg ?: course
    }

    // ---- state shown to the user ----

    private fun publish(s: State) {
        if (state.value == s) return
        state.value = s
        if (foreground) {
            runCatching { getSystemService(NotificationManager::class.java).notify(NOTIF_ID, notification(s)) }
        }
    }

    private fun shutDown() {
        runCatching { wake?.takeIf { it.isHeld }?.release() }
        stopHeading()
        loop?.cancel()
        locationJob?.cancel()
        locationJob = null
        publish(State.OFF)
        if (foreground) runCatching { stopForeground(STOP_FOREGROUND_REMOVE) }
        foreground = false
        stopSelf()
    }

    private fun startForegroundTyped(n: Notification) {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.UPSIDE_DOWN_CAKE) {
            startForeground(NOTIF_ID, n)
            return
        }
        val located = ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) ==
            PackageManager.PERMISSION_GRANTED
        if (located) {
            try {
                startForeground(
                    NOTIF_ID, n,
                    ServiceInfo.FOREGROUND_SERVICE_TYPE_CONNECTED_DEVICE or ServiceInfo.FOREGROUND_SERVICE_TYPE_LOCATION,
                )
                return
            } catch (t: Throwable) {
                Log.w(TAG, "location type refused: ${t.javaClass.simpleName}")
            }
        }
        startForeground(NOTIF_ID, n, ServiceInfo.FOREGROUND_SERVICE_TYPE_CONNECTED_DEVICE)
    }

    private fun notification(s: State): Notification {
        val nm = getSystemService(NotificationManager::class.java)
        nm.createNotificationChannel(
            NotificationChannel(CHANNEL_ID, getString(R.string.watch_channel_name), NotificationManager.IMPORTANCE_LOW),
        )
        val open = PendingIntent.getActivity(
            this, 0, Intent(this, MainActivity::class.java),
            PendingIntent.FLAG_IMMUTABLE or PendingIntent.FLAG_UPDATE_CURRENT,
        )
        return NotificationCompat.Builder(this, CHANNEL_ID)
            .setSmallIcon(R.drawable.ic_nav)
            .setContentTitle(getString(R.string.watch_channel_name))
            .setContentText(getString(s.label))
            .setOngoing(true)
            .setOnlyAlertOnce(true)
            .setShowWhen(false)
            .setContentIntent(open)
            .setPriority(NotificationCompat.PRIORITY_LOW)
            .build()
    }

    enum class State(val label: Int) {
        OFF(R.string.watch_state_off),
        CONNECTING(R.string.watch_state_connecting),
        CONNECTED(R.string.watch_state_connected),
        SHOWING(R.string.watch_state_showing),
    }

    companion object {
        private const val TAG = "VelaWatch"
        private const val ACTION_STOP = "app.vela.watch.STOP"
        private const val CHANNEL_ID = "vela_watch"
        private const val NOTIF_ID = 45
        private const val TICK_MS = 250L
        private const val WALKING_HEADING_MAX_MPS = 2.8f
        private const val HEADING_TAU_STILL_S = 0.3f
        private const val HEADING_TAU_WALKING_S = 1.5f
        private const val WALK_SMOOTH_MPS = 0.6f
        /** How often to look for the watch app while it is closed. */
        private const val IDLE_PING_MS = 3_000L
        private const val FIX_ACCURACY_M = 50f
        private const val MOVING_MPS = 1.5f
        private const val ROADS_RETRY_MS = 30_000L
        /** A new street set once the car is this far from where the last one was centred (was 250 m). */
        private const val ROADS_REFRESH_M = 120.0
        /** Street messages sent after each state. */
        private const val ROADS_PER_TICK = 6

        val state = mutableStateOf(State.OFF)

        fun start(context: Context) {
            WatchPrefs.load(context)
            if (!WatchPrefs.enabled.value || WatchPrefs.address.value == null || !DeviceService.hasBluetooth(context)) return
            runCatching {
                ContextCompat.startForegroundService(context, Intent(context, WatchService::class.java))
            }.onFailure { Log.w(TAG, "could not start the watch service") }
        }

        fun stop(context: Context) {
            if (state.value == State.OFF) return
            runCatching { context.startService(Intent(context, WatchService::class.java).setAction(ACTION_STOP)) }
        }
    }
}

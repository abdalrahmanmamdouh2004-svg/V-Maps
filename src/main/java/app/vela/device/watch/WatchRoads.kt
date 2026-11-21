package app.vela.device.watch

import app.vela.core.data.naming.RoadNameTiles
import app.vela.core.model.LatLng
import app.vela.device.RouteSimplify
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sqrt

/**
 * Nearby real street geometry for the CMF watch: every car road of Vela's map tiles round the fix, named or
 * not (compound streets, service roads), nearest first. Coordinates are metre offsets from the fix.
 */
object WatchRoads {
    private const val RADIUS_M = 700.0
    const val MAX_POINTS = 480
    const val MAX_LINES = 40

    private data class Piece(val rank: Int, val points: List<IntArray>, val nearestM: Double)

    private val major = setOf("motorway", "trunk", "primary", "secondary")
    // service roads that are only a parking aisle or a driveway are clutter on a 466 px watch
    private val clutter = setOf("parking_aisle", "driveway")
    private const val M_PER_DEG_LAT = 110_574.0
    private const val M_PER_DEG_LNG_EQ = 111_320.0

    private fun rank(r: RoadNameTiles.RoadLine): Int? = when {
        r.cls in major -> 3
        r.cls == "service" -> if (r.service in clutter) null else 1
        r.cls == "track" || r.cls == "raceway" || r.cls == "busway" -> null
        else -> 2
    }

    /** Each returned line is a separate polyline, in metres east/north from [center]. */
    fun geometry(center: LatLng, roads: List<RoadNameTiles.RoadLine>): List<List<IntArray>> {
        val dLat = RADIUS_M / 111_320.0
        val dLng = RADIUS_M / (111_320.0 * cos(Math.toRadians(center.lat)).coerceAtLeast(0.2))
        fun inside(p: LatLng) = abs(p.lat - center.lat) <= dLat && abs(p.lng - center.lng) <= dLng

        val pieces = ArrayList<Piece>()
        for (road in roads) {
            val rk = rank(road) ?: continue
            val p = road.points
            val thin = if (rk >= 3) 8.0 else 5.0
            var i = 0
            while (i < p.size) {
                fun keep(k: Int): Boolean = inside(p[k]) ||
                    (k > 0 && inside(p[k - 1])) || (k + 1 < p.size && inside(p[k + 1]))
                if (!keep(i)) {
                    i++
                    continue
                }
                var j = i
                while (j + 1 < p.size && keep(j + 1)) j++
                val thinned = RouteSimplify.thin(p.subList(i, j + 1), thin)
                val points = ArrayList<IntArray>(thinned.size)
                var nearest = Double.MAX_VALUE
                for (point in thinned) {
                    val x = ((point.lng - center.lng) * M_PER_DEG_LNG_EQ *
                        cos(Math.toRadians(center.lat))).roundToInt().coerceIn(-32767, 32767)
                    val y = ((point.lat - center.lat) * M_PER_DEG_LAT).roundToInt().coerceIn(-32767, 32767)
                    if (points.isEmpty() || points.last()[0] != x || points.last()[1] != y) {
                        points += intArrayOf(x, y)
                    }
                    nearest = minOf(nearest, sqrt(x.toDouble() * x + y.toDouble() * y))
                }
                if (points.size >= 2) pieces += Piece(rk, points, nearest)
                i = j + 1
            }
        }

        // The nearest streets win the watch's small fixed buffer; a main road wins a tie.
        val result = ArrayList<List<IntArray>>()
        var total = 0
        for (piece in pieces.sortedWith(compareBy<Piece> { (it.nearestM / 25).toInt() }.thenByDescending { it.rank }.thenBy { it.nearestM })) {
            if (result.size >= MAX_LINES) break
            val points = piece.points.take(255)
            if (points.size < 2 || total + points.size > MAX_POINTS) continue
            result += points
            total += points.size
        }
        return result
    }
}

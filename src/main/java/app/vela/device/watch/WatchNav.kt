package app.vela.device.watch

import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Messages for the watch app `vela-nav` (protocol `n1`). Every message is at most 128 bytes.
 * Positions on the wire are local metre offsets, never latitude or longitude.
 */
object WatchNav {
    const val F_ACTIVE = 1
    const val F_ARRIVED = 2
    const val F_IMPERIAL = 8
    const val F_FIX = 16
    const val F_CLOCK24 = 32
    const val F_SNAPSHOT = 64
    const val CHUNK = 30
    const val ROADS_CHUNK = 28
    const val ROADS_MAX_LINES = 40
    const val ROADS_MAX_POINTS = 480
    const val TEXT_MAX = 100
    val ASK = byteArrayOf('n'.code.toByte(), '1'.code.toByte(), 'Q'.code.toByte())
    /** The watch's Home and Work buttons. */
    val HOME = byteArrayOf('n'.code.toByte(), '1'.code.toByte(), 'H'.code.toByte())
    val WORK = byteArrayOf('n'.code.toByte(), '1'.code.toByte(), 'W'.code.toByte())

    data class State(
        val flags: Int,
        val maneuver: Int,
        val distM: Int,
        val after: Int = 0,
        val afterM: Int = 0,
        val remainingM: Int,
        val remainingS: Int,
        val limit: Int = 0,
        val x: Int,
        val y: Int,
        /** Heading in 1/65536 turns clockwise from north. */
        val course: Int,
        /** Index of the first route point still ahead. */
        val progress: Int,
    )

    fun msg(kind: Char, seq: Int, body: ByteArray): ByteArray =
        byteArrayOf('n'.code.toByte(), '1'.code.toByte(), kind.code.toByte(), seq.toByte()) + body

    private fun le(n: Int) = ByteBuffer.allocate(n).order(ByteOrder.LITTLE_ENDIAN)
    private fun u16(v: Int) = v.coerceIn(0, 0xFFFF).toShort()

    fun state(seq: Int, s: State): ByteArray = msg(
        'S', seq,
        le(24).put(s.flags.toByte()).put(s.maneuver.toByte()).putShort(u16(s.distM))
            .put(s.after.toByte()).putShort(u16(s.afterM)).putShort(u16(s.remainingM / 10))
            .putShort(u16(s.remainingS)).put(s.limit.coerceIn(0, 255).toByte())
            .putInt(s.x).putInt(s.y).putShort(s.course.toShort()).putShort(u16(s.progress)).array(),
    )

    /** Cut on a character boundary to [TEXT_MAX] bytes. */
    fun text(seq: Int, t: String): ByteArray {
        var b = t.toByteArray(Charsets.UTF_8)
        if (b.size > TEXT_MAX) {
            var n = TEXT_MAX
            while (n > 0 && (b[n].toInt() and 0xC0) == 0x80) n--
            b = b.copyOf(n)
        }
        return msg('T', seq, b)
    }

    /** A route line in metres as (kind, body) parts: begin, chunks of [CHUNK] points, end. */
    fun routeParts(id: Int, pts: List<IntArray>): List<Pair<Char, ByteArray>> {
        val max = pts.maxOfOrNull { maxOf(kotlin.math.abs(it[0]), kotlin.math.abs(it[1])) } ?: 0
        val scale = max / Short.MAX_VALUE + 1
        require(scale <= 255 && pts.size <= 512)
        val out = ArrayList<Pair<Char, ByteArray>>()
        out += 'B' to le(4).put(id.toByte()).putShort(pts.size.toShort()).put(scale.toByte()).array()
        pts.chunked(CHUNK).forEachIndexed { c, chunk ->
            val b = le(4 + 4 * chunk.size).put(id.toByte()).putShort((c * CHUNK).toShort()).put(chunk.size.toByte())
            chunk.forEach { b.putShort((it[0] / scale).toShort()).putShort((it[1] / scale).toShort()) }
            out += 'R' to b.array()
        }
        out += 'E' to le(3).put(id.toByte()).putShort(pts.size.toShort()).array()
        return out
    }

    /**
     * Nearby map roads, separate from the selected route. The centre is the current fix expressed
     * in the route's local coordinate frame; line points are local metre offsets from that fix.
     * This lets the watch rotate/project context streets with the same heading and zoom as the route.
     */
    fun roadsParts(id: Int, centre: IntArray, lines: List<List<IntArray>>): List<Pair<Char, ByteArray>> {
        require(centre.size >= 2 && lines.size <= ROADS_MAX_LINES)
        val safeLines = lines.filter { it.size in 2..255 }
        val total = safeLines.sumOf { it.size }
        require(total <= ROADS_MAX_POINTS)
        val out = ArrayList<Pair<Char, ByteArray>>()
        out += 'N' to le(11).put(id.toByte()).put(safeLines.size.toByte()).put(1.toByte())
            .putInt(centre[0]).putInt(centre[1]).array()
        safeLines.forEachIndexed { lineIndex, line ->
            line.chunked(ROADS_CHUNK).forEachIndexed { chunkIndex, chunk ->
                val start = chunkIndex * ROADS_CHUNK
                val body = le(4 + 4 * chunk.size)
                    .put(id.toByte()).put(lineIndex.toByte()).put(start.toByte()).put(chunk.size.toByte())
                chunk.forEach { point ->
                    body.putShort(point[0].coerceIn(Short.MIN_VALUE.toInt(), Short.MAX_VALUE.toInt()).toShort())
                    body.putShort(point[1].coerceIn(Short.MIN_VALUE.toInt(), Short.MAX_VALUE.toInt()).toShort())
                }
                out += 'G' to body.array()
            }
        }
        out += 'O' to le(4).put(id.toByte()).put(safeLines.size.toByte()).putShort(total.toShort()).array()
        require(out.all { it.second.size <= 128 })
        return out
    }

    /** [routeParts] numbered from [seq]. */
    fun route(seq: Int, id: Int, pts: List<IntArray>): List<ByteArray> =
        routeParts(id, pts).mapIndexed { i, (k, b) -> msg(k, seq + i, b) }
}

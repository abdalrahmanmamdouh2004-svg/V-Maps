package app.vela.device.watch

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class WatchNavRoadsTest {
    @Test
    fun contextRoadPacketsFitTheWatchChannelAndKeepSeparateLines() {
        val first = (0 until 70).map { intArrayOf(it * 3, it * 5) }
        val second = (0 until 15).map { intArrayOf(-it * 4, it * 7) }
        val parts = WatchNav.roadsParts(12, intArrayOf(1200, -3400), listOf(first, second))

        assertEquals('N', parts.first().first)
        assertEquals('O', parts.last().first)
        assertEquals(2, parts.first().second[1].toInt() and 0xFF)
        assertTrue(parts.any { it.first == 'G' })
        assertTrue(parts.all { it.second.size + 4 <= 128 })
        assertEquals(85, (parts.last().second[2].toInt() and 0xFF) or
            ((parts.last().second[3].toInt() and 0xFF) shl 8))
    }

    @Test
    fun anEmptyTileResultStillProducesACompleteClearSet() {
        val parts = WatchNav.roadsParts(2, intArrayOf(0, 0), emptyList())
        assertEquals(listOf('N', 'O'), parts.map { it.first })
        assertEquals(0, parts.last().second[1].toInt() and 0xFF)
    }
}

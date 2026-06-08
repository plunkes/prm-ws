import numpy as np


class ExploradorFronteiras:
    """
    Frontier-based exploration target selector.

    A "frontier" is a free cell (occupancy 0–LIMITE_LIVRE_MAX) that has at
    least one unknown (−1) cardinal neighbour.  The best frontier minimises:

        cost = euclidean_distance
             + |angular_error| × TURNING_PENALTY
             − INFO_GAIN_WEIGHT × normalised_unknown_density

    The info-gain term rewards frontiers that sit next to large unexplored
    areas, so the robot heads toward open space rather than hugging walls.
    All heavy lifting uses numpy – no Python loops over the map.
    """

    VALOR_DESCONHECIDO = -1
    LIMITE_LIVRE_MAX = 15      # occupancy ≤ this → free
    TURNING_PENALTY = 2.5      # cost multiplier for angular deviation
    # Frontiers closer than this (in grid cells) are ignored – they are at the
    # very edge of the last scan and the robot reaches them before SLAM updates.
    # At 0.05 m/cell → 20 cells = 1.0 m minimum travel distance.
    MIN_FRONTIER_DIST_CELLS = 20

    # Information-gain scoring: count unknown cells in a radius-R window around
    # each frontier using an integral image (O(1) per frontier after O(H×W) setup).
    # Larger window → rewards frontiers that lead to larger unexplored volumes.
    # 40 cells × 0.05 m/cell = 2.0 m — roughly half the LIDAR range.
    INFO_GAIN_WINDOW = 40
    INFO_GAIN_WEIGHT = 6.0

    def encontrar_alvo_desconhecido(self, mapa_2d, posicao_robo, yaw_atual):
        """Return (gx, gy) of the best frontier, or None if none found."""
        if mapa_2d is None or posicao_robo is None:
            return None

        livre = (mapa_2d >= 0) & (mapa_2d <= self.LIMITE_LIVRE_MAX)
        desconhecido = mapa_2d == self.VALOR_DESCONHECIDO

        # A cell is a frontier if it is free AND adjacent to at least one unknown.
        adj = (
            np.roll(desconhecido, 1, axis=0)
            | np.roll(desconhecido, -1, axis=0)
            | np.roll(desconhecido,  1, axis=1)
            | np.roll(desconhecido, -1, axis=1)
        )
        fronteira = livre & adj
        fronteira[[0, -1], :] = False
        fronteira[:, [0, -1]] = False

        ys, xs = np.where(fronteira)
        if len(xs) == 0:
            return None

        x_robo, y_robo = posicao_robo
        dx = xs.astype(np.float64) - x_robo
        dy = ys.astype(np.float64) - y_robo
        distancias = np.hypot(dx, dy)

        # Prefer frontiers that require meaningful travel.
        far = distancias >= self.MIN_FRONTIER_DIST_CELLS
        if np.any(far):
            xs, ys, dx, dy, distancias = (
                xs[far], ys[far], dx[far], dy[far], distancias[far]
            )
        else:
            idx_far = int(np.argmax(distancias))
            return int(xs[idx_far]), int(ys[idx_far])

        angulos = np.arctan2(dy, dx)
        erros = (angulos - yaw_atual + np.pi) % (2.0 * np.pi) - np.pi

        # ── Information-gain via integral image ──────────────────────────────────
        # Build a summed-area table over the unknown mask.  Then the sum of
        # unknown cells in any axis-aligned rectangle is a single O(1) lookup.
        unk = desconhecido.astype(np.float32)
        H, W = unk.shape
        # ii[i, j] = sum of unk[0:i, 0:j]  (1-indexed, first row/col = 0)
        ii = np.zeros((H + 1, W + 1), dtype=np.float32)
        ii[1:, 1:] = np.cumsum(np.cumsum(unk, axis=0), axis=1)

        r = self.INFO_GAIN_WINDOW
        y0 = np.maximum(ys - r,     0)
        y1 = np.minimum(ys + r + 1, H)
        x0 = np.maximum(xs - r,     0)
        x1 = np.minimum(xs + r + 1, W)
        info_gains = ii[y1, x1] - ii[y0, x1] - ii[y1, x0] + ii[y0, x0]

        max_gain = info_gains.max()
        if max_gain > 0:
            info_gains = info_gains / max_gain   # normalise to [0, 1]

        # Lower cost = better.  Info-gain term is subtracted so high-gain
        # frontiers (leading to large unknown areas) are strongly preferred.
        custos = (distancias
                  + np.abs(erros) * self.TURNING_PENALTY
                  - self.INFO_GAIN_WEIGHT * info_gains)

        idx = int(np.argmin(custos))
        return int(xs[idx]), int(ys[idx])

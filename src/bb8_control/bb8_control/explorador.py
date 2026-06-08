import numpy as np


class ExploradorFronteiras:
    """
    Frontier-based exploration target selector.

    A "frontier" is a free cell (occupancy 0–LIMITE_LIVRE_MAX) that has at
    least one unknown (−1) cardinal neighbour.  The best frontier minimises:

        cost = euclidean_distance + |angular_error| × TURNING_PENALTY

    so the robot strongly prefers moving forward over turning.
    All heavy lifting uses numpy – no Python loops over the map.
    """

    VALOR_DESCONHECIDO = -1
    LIMITE_LIVRE_MAX = 15     # occupancy ≤ this → free
    TURNING_PENALTY = 3.0     # cost multiplier for angular deviation
    # Frontiers closer than this (in grid cells) are ignored – they are at the
    # very edge of the last scan and the robot reaches them before SLAM updates.
    # At 0.05 m/cell → 20 cells = 1.0 m minimum travel distance.
    MIN_FRONTIER_DIST_CELLS = 20

    def encontrar_alvo_desconhecido(self, mapa_2d, posicao_robo, yaw_atual):
        """Return (gx, gy) of the best frontier, or None if none found."""
        if mapa_2d is None or posicao_robo is None:
            return None

        livre = (mapa_2d >= 0) & (mapa_2d <= self.LIMITE_LIVRE_MAX)
        desconhecido = mapa_2d == self.VALOR_DESCONHECIDO

        # A cell is a frontier if it is free AND adjacent to at least one unknown.
        # np.roll wraps at borders; we zero-out the border strip afterwards so
        # no phantom frontiers are created at map edges.
        adj = (
            np.roll(desconhecido, 1, axis=0)   # unknown above?
            | np.roll(desconhecido, -1, axis=0)  # unknown below?
            | np.roll(desconhecido,  1, axis=1)  # unknown left?
            | np.roll(desconhecido, -1, axis=1)  # unknown right?
        )
        fronteira = livre & adj
        # Clear 1-cell border to avoid wrap-around artefacts
        fronteira[[0, -1], :] = False
        fronteira[:, [0, -1]] = False

        ys, xs = np.where(fronteira)
        if len(xs) == 0:
            return None

        x_robo, y_robo = posicao_robo
        dx = xs.astype(np.float64) - x_robo
        dy = ys.astype(np.float64) - y_robo
        distancias = np.hypot(dx, dy)

        # Prefer frontiers that require meaningful travel.  If some exist beyond
        # the minimum distance, restrict the candidate set to those; otherwise
        # fall back to whichever frontier is farthest (map just started building).
        far = distancias >= self.MIN_FRONTIER_DIST_CELLS
        if np.any(far):
            xs, ys, dx, dy, distancias = (
                xs[far], ys[far], dx[far], dy[far], distancias[far]
            )
        else:
            # All frontiers are close – pick the farthest one to at least move
            idx_far = int(np.argmax(distancias))
            return int(xs[idx_far]), int(ys[idx_far])

        angulos = np.arctan2(dy, dx)
        erros = (angulos - yaw_atual + np.pi) % (2.0 * np.pi) - np.pi
        custos = distancias + np.abs(erros) * self.TURNING_PENALTY

        idx = int(np.argmin(custos))
        return int(xs[idx]), int(ys[idx])

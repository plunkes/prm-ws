from bb8_control.explorador import ExploradorFronteiras
from bb8_control.d_star_lite import DStarLitePersonalizado
import numpy as np


class GerenciadorMissao:
    """
    FSM (Finite State Machine) for autonomous robot exploration and flag collection.
    
    States:
    - EXPLORANDO: Explores unknown areas using D* Lite with reactive fallback
    - BANDEIRA_DETECTADA: Flag detected, initiating approach
    - NAVIGANDO_PARA_BANDEIRA: Navigating to flag using D* Lite
    - PROCURANDO_BANDEIRA: Lost flag, doing 360° rotation
    - POSICIONANDO_PARA_COLETA: Fine-tuning alignment for collection
    """

    def __init__(self):
        self.ESTADO_ATUAL = "EXPLORANDO"

        self.explorador = ExploradorFronteiras()
        self.navegador = DStarLitePersonalizado()

        self.caminho_atual = []
        self.alvo_atual = None
        self.ultima_posicao_robo = None
        self.ultima_posicao_bandeira = None

        # --- VARIÁVEIS DOS NOVOS ESTADOS ---
        self.ticks_procurando = 0
        # Ponto ótimo para não entrar no ponto cego do LIDAR e não colar na câmera
        self.DISTANCIA_COLETA = 0.4
        
        # Exploration fallback heuristic parameters
        self.UNKNOWN_CELL_VALUE = -1
        self.FRONTIER_SEARCH_RADIUS = 10  # cells to search for unknowns
        self.exploration_timeout = 0  # counter for exploration state timeout

    def atualizar_estado_e_caminho(
        self,
        mapa_2d,
        posicao_robo,
        bandeira_detectada,
        pos_bandeira_grid,
        distancia_frente,
        yaw_atual,
    ):
        """
        Update FSM state and compute the next movement target.
        
        Args:
            mapa_2d: OccupancyGrid as numpy array
            posicao_robo: (x, y) robot position in grid coordinates
            bandeira_detectada: boolean flag detection status
            pos_bandeira_grid: (x, y) flag position in grid if detected
            distancia_frente: distance to nearest obstacle (m)
            yaw_atual: robot heading (rad)
        
        Returns:
            Next grid cell to move towards, or fallback velocity if needed
        """
        if mapa_2d is None or posicao_robo is None:
            return None

        # REGRAS DE TRANSIÇÃO DE ESTADOS
        if self.ESTADO_ATUAL == "EXPLORANDO":
            if bandeira_detectada:
                print(
                    "[FSM] Flag detected! Transitioning to BANDEIRA_DETECTADA."
                )
                self.ESTADO_ATUAL = "BANDEIRA_DETECTADA"
                self.alvo_atual = None  # Reset to restart pathfinding
                self.exploration_timeout = 0

        elif self.ESTADO_ATUAL == "BANDEIRA_DETECTADA":
            if not bandeira_detectada:
                print(
                    "[FSM] Flag lost during detection! Returning to EXPLORANDO."
                )
                self.ESTADO_ATUAL = "EXPLORANDO"
                self.alvo_atual = None
            else:
                # Flag confirmed, move to navigation state
                print(
                    "[FSM] Flag confirmed. Transitioning to NAVIGANDO_PARA_BANDEIRA."
                )
                self.ESTADO_ATUAL = "NAVIGANDO_PARA_BANDEIRA"
                self.alvo_atual = pos_bandeira_grid
                self.ultima_posicao_bandeira = pos_bandeira_grid

        elif self.ESTADO_ATUAL == "NAVIGANDO_PARA_BANDEIRA":
            if not bandeira_detectada:
                print(
                    "[FSM] Lost flag during navigation! Entering PROCURANDO_BANDEIRA."
                )
                self.ESTADO_ATUAL = "PROCURANDO_BANDEIRA"
                self.ticks_procurando = 0
            elif distancia_frente <= self.DISTANCIA_COLETA:
                print(
                    "[FSM] Ideal distance reached! Transitioning to POSICIONANDO_PARA_COLETA."
                )
                self.ESTADO_ATUAL = "POSICIONANDO_PARA_COLETA"

        elif self.ESTADO_ATUAL == "PROCURANDO_BANDEIRA":
            if bandeira_detectada:
                print(
                    "[FSM] Flag re-acquired during rotation! Resuming navigation."
                )
                self.ESTADO_ATUAL = "NAVIGANDO_PARA_BANDEIRA"
            else:
                self.ticks_procurando += 1
                # 360° rotation: 12.5s at 0.5 rad/s, at 20Hz = ~251 ticks, margin 260
                if self.ticks_procurando > 260:
                    print(
                        "[FSM] 360° rotation complete. Flag not found. Returning to EXPLORANDO."
                    )
                    self.ESTADO_ATUAL = "EXPLORANDO"
                    self.alvo_atual = None
                    self.exploration_timeout = 0

        elif self.ESTADO_ATUAL == "POSICIONANDO_PARA_COLETA":
            if not bandeira_detectada:
                # Flag passed or collision occurred
                self.ESTADO_ATUAL = "PROCURANDO_BANDEIRA"
                self.ticks_procurando = 0

        # AÇÕES DE ACORDO COM O ESTADO
        if self.ESTADO_ATUAL == "EXPLORANDO":
            # Try to find next frontier
            if self.alvo_atual is None or posicao_robo == self.alvo_atual:
                novo_alvo = self.explorador.encontrar_alvo_desconhecido(
                    mapa_2d, posicao_robo, yaw_atual
                )
                if novo_alvo:
                    self.alvo_atual = novo_alvo
                    self.navegador.inicializar_planejamento(
                        mapa_2d, posicao_robo, self.alvo_atual
                    )
                    self.exploration_timeout = 0
                else:
                    self.exploration_timeout += 1

        elif self.ESTADO_ATUAL == "BANDEIRA_DETECTADA":
            # Brief state - transition happens in next cycle
            pass

        elif self.ESTADO_ATUAL == "NAVIGANDO_PARA_BANDEIRA":
            if pos_bandeira_grid is not None:
                # Update target if flag moved
                if self.alvo_atual != pos_bandeira_grid:
                    self.alvo_atual = pos_bandeira_grid
                    self.ultima_posicao_bandeira = pos_bandeira_grid
                    self.navegador.inicializar_planejamento(
                        mapa_2d, posicao_robo, self.alvo_atual
                    )

        # CALCULAR ROTA COM D* LITE (para EXPLORANDO e NAVIGANDO_PARA_BANDEIRA)
        if (
            self.ESTADO_ATUAL in ["EXPLORANDO", "NAVIGANDO_PARA_BANDEIRA"]
            and self.alvo_atual
        ):
            # Update robot position in D* Lite
            if self.ultima_posicao_robo is not None:
                self.navegador.km += self.navegador.heuristica(
                    self.ultima_posicao_robo, posicao_robo
                )
            self.navegador.inicio = posicao_robo
            self.ultima_posicao_robo = posicao_robo

            self.navegador.mapa = mapa_2d
            self.navegador.calcular_caminho_mais_curto()
            self.caminho_atual = self.navegador.extrair_caminho()

            if len(self.caminho_atual) > 1:
                return self.caminho_atual[1]

        return None

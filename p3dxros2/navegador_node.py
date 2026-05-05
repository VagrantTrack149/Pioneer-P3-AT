
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan

import numpy as np
import cv2 as cv
import heapq
import random
import math

RADIO_ROBOT     = 3      # celdas de inflado (~0.15 m con res 0.05)
HORIZON_CHECK   = 8      # pasos adelante para detectar obstáculos
Kv              = 0.6    # ganancia velocidad lineal
Kh              = 1.8    # ganancia velocidad angular
DIST_UMBRAL     = 0.30   # distancia waypoint alcanzado (m)
MAX_RETROCESO   = 25     # pasos de retroceso
VEL_RETROCESO   = -0.15  # velocidad retroceso (m/s)
VEL_MAX         = 0.35   # velocidad lineal máxima (m/s)
TAMANO_SECTOR   = 5      # ancho sector en celdas

# Anti-atasco
UMBRAL_DIST_ATASCO  = 0.05   # si se mueve menos de esto en X segundos -- atasco
TIEMPO_ATASCO       = 8.0    # segundos sin moverse = atasco
DURACION_ESCAPE     = 3.0    # segundos de movimiento aleatorio al escapar
MAX_INTENTOS_FRONT  = 5      # intentos fallidos de frontera --> exploración aleatoria
DURACION_ALETORIA   = 5.0    # segundos de movimiento aleatorio cuando no hay frontera


# A* ─

def expandir_obstaculos(grid, radio_px):
    mask = np.where(grid == 100, 255, 0).astype(np.uint8)
    k = cv.getStructuringElement(
        cv.MORPH_ELLIPSE, (radio_px * 2 + 1, radio_px * 2 + 1))
    inflado = cv.dilate(mask, k)
    g = grid.copy()
    g[inflado == 255] = 100
    return g


def a_estrella(mapa, inicio, meta):
    """mapa: 0=libre, otro=bloqueado"""
    rows, cols = mapa.shape
    open_set = []
    heapq.heappush(open_set, (0, 0, inicio))
    came_from = {}
    g_score = {inicio: 0}
    nb8 = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    while open_set:
        _, g, cur = heapq.heappop(open_set)
        if cur == meta:
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            return path[::-1]
        for dx, dy in nb8:
            nb = (cur[0]+dx, cur[1]+dy)
            if not (0 <= nb[0] < rows and 0 <= nb[1] < cols):
                continue
            if mapa[nb[0], nb[1]] != 0:
                continue
            c = g + (1.414 if dx and dy else 1.0)
            if nb not in g_score or c < g_score[nb]:
                came_from[nb] = cur
                g_score[nb] = c
                h = math.hypot(nb[0]-meta[0], nb[1]-meta[1])
                heapq.heappush(open_set, (c+h, c, nb))
    return None


def celda_libre(mapa_astar, p, radio=2):
    rows, cols = mapa_astar.shape
    for dr in range(-radio, radio+1):
        for dc in range(-radio, radio+1):
            nr, nc = p[0]+dr, p[1]+dc
            if 0 <= nr < rows and 0 <= nc < cols:
                if mapa_astar[nr, nc] != 0:
                    return False
    return True


def buscar_frontera(mapa_ros, mapa_astar, pos):
    """Fronteras = celdas libres con vecino desconocido (-1)."""
    rows, cols = mapa_ros.shape
    libres = np.argwhere(mapa_ros == 0)
    candidatos = []
    for p in libres:
        rr, cc = int(p[0]), int(p[1])
        if mapa_astar[rr, cc] != 0:
            continue
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = rr+dr, cc+dc
            if 0 <= nr < rows and 0 <= nc < cols and mapa_ros[nr,nc] == -1:
                dist = math.hypot(rr-pos[0], cc-pos[1])
                candidatos.append((dist, (rr, cc)))
                break
    if not candidatos:
        return None
    candidatos.sort(key=lambda x: x[0])
    # Prueba los más cercanos primero, luego algunos lejanos
    for _, punto in candidatos[:15]:
        if a_estrella(mapa_astar, pos, punto):
            return punto
    # Si los cercanos fallan, intenta lejanos
    for _, punto in candidatos[15:30]:
        if a_estrella(mapa_astar, pos, punto):
            return punto
    return None


def punto_aleatorio_libre(mapa_astar, pos, min_dist=5, max_dist=30):
    """Genera un punto aleatorio libre al que se pueda llegar."""
    rows, cols = mapa_astar.shape
    libres = np.argwhere(mapa_astar == 0)
    if len(libres) == 0:
        return None
    np.random.shuffle(libres)
    for p in libres[:50]:
        rr, cc = int(p[0]), int(p[1])
        d = math.hypot(rr-pos[0], cc-pos[1])
        if min_dist <= d <= max_dist and celda_libre(mapa_astar, (rr,cc), 2):
            if a_estrella(mapa_astar, pos, (rr,cc)):
                return (rr, cc)
    return None


#  Nodo ─

class NavegadorNode(Node):

    def __init__(self):
        super().__init__('navegador_autonomo')

        # Estado robot
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw   = 0.0
        self.odom_ok = False

        # Mapa
        self.mapa_ros  = None
        self.mapa_info = None

        # Navegación
        self.objetivo_actual = None
        self.camino_actual   = []
        self.indice_wp       = 0

        self.modo_retroceso  = False
        self.pasos_retroceso = 0

        self.explorando = True
        self.vigilancia = False
        self.puntos_vigilancia = []
        self.indice_vig = 0

        self.puntos_seguros   = {}
        self.puntos_visitados = []

        # Anti-atasco
        self.pos_anterior      = None
        self.t_ultima_posicion = self.get_clock().now()
        self.en_escape         = False
        self.t_inicio_escape   = None
        self.vel_escape        = (0.0, 0.0)

        # Exploración aleatoria cuando no hay frontera
        self.intentos_frontera = 0
        self.en_aleatorio      = False
        self.t_inicio_aleatorio= None

        # Giro inicial para generar mapa
        self.t_inicio = self.get_clock().now()
        self.GIRO_INICIAL_SEG = 4.0   # segundos girando al inicio

        # QoS mapa
        qos_mapa = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.sub_odom = self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self._cb_scan, 10)
        self.sub_map  = self.create_subscription(OccupancyGrid, '/map', self._cb_map, qos_mapa)
        self.pub_cmd  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer    = self.create_timer(0.1, self._paso)

        self.get_logger().info('Navegador v2 iniciado.')

    #  Callbacks 

    def _cb_odom(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0*(q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)
        self.odom_ok = True

    def _cb_scan(self, msg):
        pass

    def _cb_map(self, msg):
        self.mapa_info = msg.info
        arr = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        self.mapa_ros = arr.astype(np.int16)

    #  Conversiones ─

    def mundo_a_celda(self, x, y):
        if self.mapa_info is None:
            return None
        res = self.mapa_info.resolution
        ox  = self.mapa_info.origin.position.x
        oy  = self.mapa_info.origin.position.y
        col = int((x - ox) / res)
        row = int((y - oy) / res)
        h, w = self.mapa_ros.shape
        if 0 <= row < h and 0 <= col < w:
            return (row, col)
        return None

    def celda_a_mundo(self, row, col):
        res = self.mapa_info.resolution
        ox  = self.mapa_info.origin.position.x
        oy  = self.mapa_info.origin.position.y
        return col*res + ox + res/2.0, row*res + oy + res/2.0

    #  Publicar velocidad 

    def _vel(self, v, omega):
        msg = Twist()
        msg.linear.x  = float(np.clip(v, -VEL_MAX, VEL_MAX))
        msg.angular.z = float(np.clip(omega, -2.5, 2.5))
        self.pub_cmd.publish(msg)

    def _detener(self):
        try:
            self.pub_cmd.publish(Twist())
        except Exception:
            pass

    #  Anti-atasco ─

    def _actualizar_atasco(self):
        """Devuelve True si el robot está atascado."""
        ahora = self.get_clock().now()
        pos = (self.pos_x, self.pos_y)

        if self.pos_anterior is None:
            self.pos_anterior = pos
            self.t_ultima_posicion = ahora
            return False

        dist = math.hypot(pos[0]-self.pos_anterior[0], pos[1]-self.pos_anterior[1])
        if dist > UMBRAL_DIST_ATASCO:
            self.pos_anterior = pos
            self.t_ultima_posicion = ahora
            return False

        dt = (ahora - self.t_ultima_posicion).nanoseconds / 1e9
        return dt > TIEMPO_ATASCO

    #  Paso principal 

    def _paso(self):
        if not self.odom_ok:
            return

        ahora = self.get_clock().now()
        elapsed = (ahora - self.t_inicio).nanoseconds / 1e9

        #  FASE 0: Giro inicial para generar mapa 
        # Siempre gira los primeros segundos, tenga o no mapa
        if elapsed < self.GIRO_INICIAL_SEG:
            self._vel(0.0, 1.0)
            return

        #  Sin mapa todavía: gira lento para generarlo ─
        if self.mapa_ros is None:
            self._vel(0.0, 0.5)
            return

        # Posición en celdas
        pos_celda = self.mundo_a_celda(self.pos_x, self.pos_y)
        if pos_celda is None:
            # Robot fuera del mapa: avanza recto para entrar
            self._vel(0.15, 0.0)
            return

        # Mapa de planificación
        mapa_exp  = expandir_obstaculos(self.mapa_ros, RADIO_ROBOT)
        mapa_astar = np.where(mapa_exp == 0, 0, 1).astype(np.int8)

        # Registrar punto seguro
        if celda_libre(mapa_astar, pos_celda, 2):
            sector = (pos_celda[0]//TAMANO_SECTOR, pos_celda[1]//TAMANO_SECTOR)
            self.puntos_seguros[sector] = pos_celda

        self.puntos_visitados.append(pos_celda)
        if len(self.puntos_visitados) > 600:
            self.puntos_visitados.pop(0)

        #  ESCAPE DE ATASCO 
        atascado = self._actualizar_atasco()
        if atascado and not self.en_escape and not self.modo_retroceso:
            self.get_logger().warn('¡Atasco detectado! Iniciando escape.')
            self.en_escape       = True
            self.t_inicio_escape = ahora
            self.objetivo_actual = None
            self.camino_actual   = []
            self.indice_wp       = 0
            # Giro aleatorio + retroceso
            sentido = random.choice([-1.0, 1.0])
            self.vel_escape = (-0.1, sentido * 1.5)
            self.pos_anterior = None  # reset anti-atasco

        if self.en_escape:
            dt_escape = (ahora - self.t_inicio_escape).nanoseconds / 1e9
            if dt_escape < DURACION_ESCAPE:
                self._vel(*self.vel_escape)
                return
            else:
                self.get_logger().info('Escape completado.')
                self.en_escape = False
                self._detener()
                return

        #  RETROCESO ─
        if self.modo_retroceso:
            self._vel(VEL_RETROCESO, 0.0)
            self.pasos_retroceso += 1
            if self.pasos_retroceso >= MAX_RETROCESO:
                self.modo_retroceso  = False
                self.pasos_retroceso = 0
                self.objetivo_actual = None
                self.camino_actual   = []
                self.indice_wp       = 0
                self._detener()
            return

        #  VIGILANCIA 
        if self.vigilancia:
            self._modo_vigilancia(pos_celda, mapa_astar)
            return

        #  EXPLORACIÓN ALEATORIA (cuando no hay frontera) 
        if self.en_aleatorio:
            dt_al = (ahora - self.t_inicio_aleatorio).nanoseconds / 1e9
            if dt_al < DURACION_ALETORIA:
                # Si tiene objetivo aleatorio, síguelo; si no, gira
                if self.camino_actual and self.indice_wp < len(self.camino_actual):
                    self._seguir_camino(pos_celda, mapa_astar)
                else:
                    self._vel(0.2, random.uniform(-0.8, 0.8))
                return
            else:
                self.en_aleatorio      = False
                self.intentos_frontera = 0
                self.objetivo_actual   = None
                self.camino_actual     = []
                self.get_logger().info('Fin exploración aleatoria, volviendo a fronteras.')
                return

        #  EXPLORACIÓN: buscar frontera 
        if self.explorando and self.objetivo_actual is None:
            objetivo = buscar_frontera(self.mapa_ros, mapa_astar, pos_celda)

            if objetivo is None:
                self.intentos_frontera += 1
                self.get_logger().info(
                    f'Sin frontera (intento {self.intentos_frontera}/{MAX_INTENTOS_FRONT})')

                if self.intentos_frontera >= MAX_INTENTOS_FRONT:
                    # ¿Terminó la exploración o está atascado?
                    self.get_logger().info(
                        '¡Exploración terminada! Pasando a vigilancia.')
                    self.explorando = False
                    self.vigilancia = True
                    paso = max(1, len(self.puntos_visitados)//30)
                    vistos = []
                    for i in range(0, len(self.puntos_visitados), paso):
                        p = self.puntos_visitados[i]
                        if not vistos or math.hypot(
                                p[0]-vistos[-1][0], p[1]-vistos[-1][1]) > 3:
                            vistos.append(p)
                    self.puntos_vigilancia = vistos
                    self._detener()
                    return

                # Exploración aleatoria para descubrir zona
                self.get_logger().info('Movimiento aleatorio para descubrir zona.')
                punto_al = punto_aleatorio_libre(mapa_astar, pos_celda)
                self.en_aleatorio       = True
                self.t_inicio_aleatorio = ahora
                if punto_al:
                    camino = a_estrella(mapa_astar, pos_celda, punto_al)
                    if camino:
                        self.objetivo_actual = punto_al
                        self.camino_actual   = camino
                        self.indice_wp       = 0
                return

            # Frontera encontrada
            self.intentos_frontera = 0
            camino = a_estrella(mapa_astar, pos_celda, objetivo)
            if camino:
                self.objetivo_actual = objetivo
                self.camino_actual   = camino
                self.indice_wp       = 0
            else:
                self.get_logger().warn('Sin camino a frontera, retrocediendo.')
                self.modo_retroceso = True
            return

        #  Seguir camino ─
        if not self.camino_actual or self.indice_wp >= len(self.camino_actual):
            self.objetivo_actual = None
            return

        # Verificar obstáculo en horizonte
        fin_h = min(self.indice_wp + HORIZON_CHECK, len(self.camino_actual))
        hay_obs = any(
            mapa_astar[self.camino_actual[hi][0], self.camino_actual[hi][1]] != 0
            for hi in range(self.indice_wp, fin_h)
        )

        if hay_obs:
            self.get_logger().info('Obstáculo en horizonte, replanificando.')
            nuevo = a_estrella(mapa_astar, pos_celda, self.objetivo_actual)
            if nuevo:
                self.camino_actual = nuevo
                self.indice_wp     = 0
            else:
                self.get_logger().warn('Sin replan, retrocediendo.')
                self.modo_retroceso  = True
                self.objetivo_actual = None
                self.camino_actual   = []
            return

        self._seguir_camino(pos_celda, mapa_astar)

    #  Controlador hacia waypoint 

    def _seguir_camino(self, pos_celda, mapa_astar):
        if not self.camino_actual or self.indice_wp >= len(self.camino_actual):
            self.objetivo_actual = None
            self._detener()
            return

        wp = self.camino_actual[self.indice_wp]
        wx, wy = self.celda_a_mundo(wp[0], wp[1])
        dx = wx - self.pos_x
        dy = wy - self.pos_y
        dist = math.hypot(dx, dy)

        if dist < DIST_UMBRAL:
            self.indice_wp += 1
            if self.indice_wp >= len(self.camino_actual):
                self.objetivo_actual = None
                self._detener()
            return

        x_r =  dx * math.cos(self.yaw) + dy * math.sin(self.yaw)
        y_r = -dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
        ang_err = math.atan2(y_r, x_r)

        v     = 0.0 if abs(ang_err) > 0.9 else np.clip(Kv * dist, 0.05, VEL_MAX)
        omega = Kh * ang_err
        self._vel(v, omega)

    #  Modo vigilancia ─

    def _modo_vigilancia(self, pos_celda, mapa_astar):
        if not self.puntos_vigilancia:
            self._detener()
            return

        if self.objetivo_actual is None:
            meta = self.puntos_vigilancia[
                self.indice_vig % len(self.puntos_vigilancia)]
            camino = a_estrella(mapa_astar, pos_celda, meta)
            if camino:
                self.objetivo_actual = meta
                self.camino_actual   = camino
                self.indice_wp       = 0
            else:
                self.indice_vig += 1
            return

        if not self.camino_actual or self.indice_wp >= len(self.camino_actual):
            self.get_logger().info('Punto vigilancia alcanzado.')
            self.indice_vig     += 1
            self.objetivo_actual = None
            self.camino_actual   = []
            self.indice_wp       = 0
            self._detener()
            return

        self._seguir_camino(pos_celda, mapa_astar)


def main(args=None):
    rclpy.init(args=args)
    node = NavegadorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._detener()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


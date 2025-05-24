#!/usr/bin/env python3
# multi_shape_path_node.py  (circle, ellipse, square, triangle, star, spike)
import rclpy, math, time, bisect
from rclpy.node import Node
from geometry_msgs.msg import Point

class MultiShapePath(Node):
    def __init__(self):
        super().__init__('multi_shape_path')

        # ── Parámetros ─────────────────────────────────────────
        self.declare_parameter('shape',      'star')    # circle|ellipse|square|triangle|star|spike
        self.declare_parameter('points',      5)          # puntas estrella/spike
        self.declare_parameter('radius',      2.0)        # radio exterior
        self.declare_parameter('radius_x',    2.0)        # semieje mayor elipse
        self.declare_parameter('radius_y',    1.0)        # semieje menor elipse
        self.declare_parameter('width',       4.0)        # lado cuadrado / triángulo
        self.declare_parameter('lin_speed',   0.20)       # m/s (máx robot)
        self.declare_parameter('center_x',    0.0)
        self.declare_parameter('center_y',    0.0)

        # ── Lectura ───────────────────────────────────────────
        self.shape = self.get_parameter('shape').value.lower()
        self.n     = max(3, int(self.get_parameter('points').value))
        self.R     = self.get_parameter('radius').value
        self.a     = self.get_parameter('radius_x').value
        self.b     = self.get_parameter('radius_y').value
        self.w     = self.get_parameter('width').value
        self.v     = min(self.get_parameter('lin_speed').value, 0.20)
        self.cx    = self.get_parameter('center_x').value
        self.cy    = self.get_parameter('center_y').value

        # ── Selección de trayectoria ──────────────────────────
        if   self.shape == 'circle':    self._setup_circle()
        elif self.shape == 'ellipse':   self._setup_ellipse()
        elif self.shape == 'square':    self._setup_square()
        elif self.shape == 'triangle':  self._setup_triangle()
        elif self.shape == 'star':      self._setup_star()
        elif self.shape == 'spike':     self._setup_spike()
        else:
            raise ValueError("shape inválido")

        self.get_logger().info(
            f"shape={self.shape} | v={self.v:.2f} m/s | T={self.T_total:.2f} s")

        # ── ROS I/O ───────────────────────────────────────────
        self.pub  = self.create_publisher(Point, '/next_point', 10)
        self.msg  = Point()
        self.t0   = time.time()
        self.create_timer(0.02, self._update)  # 50 Hz

    # ---------- Circle ----------
    def _setup_circle(self):
        self.T_total = 2*math.pi*self.R / self.v
        self._gen = lambda t: (
            self.cx + self.R*math.cos(2*math.pi*t/self.T_total),
            self.cy + self.R*math.sin(2*math.pi*t/self.T_total)
        )

    # ---------- Ellipse (const-speed con tabla arco-longitud) ----------
    def _setup_ellipse(self, N=720):
        thetas = [2*math.pi*i/N for i in range(N+1)]
        xs, ys, s = [], [], [0.0]
        for i, th in enumerate(thetas):
            xs.append(self.cx + self.a*math.cos(th))
            ys.append(self.cy + self.b*math.sin(th))
            if i:
                s.append(s[-1] + math.hypot(xs[i]-xs[i-1], ys[i]-ys[i-1]))
        self.perim, self.table_s = s[-1], s
        self.table_x, self.table_y = xs, ys
        self.T_total = self.perim / self.v
        self._gen = self._gen_ellipse

    def _gen_ellipse(self, t):
        s_target = (t % self.T_total)*self.v
        i = bisect.bisect_left(self.table_s, s_target)
        if i == 0: return self.table_x[0], self.table_y[0]
        s0, s1 = self.table_s[i-1], self.table_s[i]
        tau = (s_target - s0) / (s1 - s0)
        x = (1-tau)*self.table_x[i-1] + tau*self.table_x[i]
        y = (1-tau)*self.table_y[i-1] + tau*self.table_y[i]
        return x, y

    # ---------- Square ----------
    def _setup_square(self):
        half = self.w/2
        v = [(self.cx-half, self.cy-half), (self.cx+half, self.cy-half),
             (self.cx+half, self.cy+half), (self.cx-half, self.cy+half)]
        self._build_polygon(v)

    # ---------- Triangle (equilátero) ----------
    def _setup_triangle(self):
        s  = self.w
        h  = s*math.sqrt(3)/2
        v = [(self.cx,         self.cy + 2*h/3),   # vértice superior
             (self.cx - s/2,   self.cy - h/3),     # base izquierda
             (self.cx + s/2,   self.cy - h/3)]     # base derecha
        self._build_polygon(v)

    # ---------- Star (hueca) ----------
    def _setup_star(self):
        rin = self.R*math.sin(math.pi/self.n)/math.sin(3*math.pi/self.n)
        v = []
        for k in range(self.n):
            ao, ai = 2*math.pi*k/self.n, 2*math.pi*k/self.n + math.pi/self.n
            v.extend([(self.cx+self.R*math.cos(ao),  self.cy+self.R*math.sin(ao)),
                      (self.cx+rin*math.cos(ai),     self.cy+rin*math.sin(ai))])
        self._build_polygon(v)

    # ---------- Spike ----------
    def _setup_spike(self):
        self.edges   = 2*self.n
        self.T_edge  = self.R/self.v
        self.T_total = self.edges*self.T_edge
        self._gen = self._gen_spike

    def _gen_spike(self, t):
        seg = int((t % self.T_total)//self.T_edge)
        tau = ((t % self.T_total) % self.T_edge)/self.T_edge
        k   = seg//2
        tip = (self.cx+self.R*math.cos(2*math.pi*k/self.n),
               self.cy+self.R*math.sin(2*math.pi*k/self.n))
        p0, p1 = ((self.cx, self.cy), tip) if seg%2==0 else (tip, (self.cx, self.cy))
        return (1-tau)*p0[0]+tau*p1[0], (1-tau)*p0[1]+tau*p1[1]

    # ---------- Herramienta polígono genérico ----------
    def _build_polygon(self, vertices):
        self.vertices = vertices
        L = [math.dist(vertices[i], vertices[(i+1)%len(vertices)])
             for i in range(len(vertices))]
        self.cum_t = [0.0]
        for d in L: self.cum_t.append(self.cum_t[-1]+d/self.v)
        self.T_total = self.cum_t[-1]
        self._gen = self._gen_polygon

    def _gen_polygon(self, t):
        tm = t % self.T_total
        idx = bisect.bisect_right(self.cum_t, tm)-1
        tau = (tm - self.cum_t[idx]) / (self.cum_t[idx+1]-self.cum_t[idx])
        p0, p1 = self.vertices[idx], self.vertices[(idx+1)%len(self.vertices)]
        return (1-tau)*p0[0]+tau*p1[0], (1-tau)*p0[1]+tau*p1[1]

    # ---------- Publicación ----------
    def _update(self):
        self.msg.x, self.msg.y = self._gen(time.time()-self.t0)
        self.msg.z = 0.0
        self.pub.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MultiShapePath())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

// Copyright 2026 Juan José Martínez Medina
// Licensed under the Apache License, Version 2.0
//
// Vista del mapa: dibuja el plano del mundo y/o el OccupancyGrid de SLAM, el
// robot, el scan del LiDAR, la ruta planificada, la meta y los waypoints.
// Convierte los clics en coordenadas del mundo (marco map) para enviar metas.
//
// La vista se puede girar. Por defecto el eje x apunta hacia arriba y el y a
// la izquierda, igual que la camara cenital por defecto de Gazebo.

const COLORS = {
  free: [26, 61, 97],        // --sheet-2
  wall: [232, 239, 245],     // --paper
  grid: 'rgba(143, 171, 196, 0.13)',
  axis: 'rgba(143, 171, 196, 0.45)',
  robot: '#ff9f43',
  scan: 'rgba(255, 64, 64, 0.95)',    // rojo: impactos del LiDAR
  plan: '#e8eff5',
  route: '#f5d76e',
  ink: '#0d2236',
};

// Huella del robot respecto a base_footprint (igual que en nav2_params.yaml)
const FOOTPRINT = [[0.33, 0.21], [0.33, -0.21], [-0.13, -0.21], [-0.13, 0.21]];

const DRAG_MIN_PX = 8;

export class MapView {
  constructor(canvas, { onPick, onCursor }) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.onPick = onPick;
    this.onCursor = onCursor;

    // Centro de la vista (m), escala (px/m) y giro de la vista (rad)
    this.view = { cx: 0, cy: 0, scale: 50, rot: Math.PI / 2 };
    this.layers = { plano: true, slam: false };
    this.world = null;        // plano del mundo: [{type, x, y, ...}]
    this.worldBounds = null;
    this.mapImage = null;
    this.mapInfo = null;
    this.robot = null;        // {x, y, yaw}
    this.scanPoints = [];     // [[x, y], ...] en el marco map
    this.showScan = true;
    this.plan = [];
    this.goal = null;         // {x, y, yaw}
    this.route = [];          // [{x, y, yaw}]
    this.routeProgress = -1;  // indice del waypoint actual
    this.drag = null;         // {x, y, yaw} mientras se elige orientacion
    this.follow = false;
    this.fitted = false;
    this.dirty = true;

    new ResizeObserver(() => this.resize()).observe(canvas);
    this.bindPointer();
    const loop = () => {
      if (this.dirty) this.draw();
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  }

  // ------------------------------------------------------------ Datos
  setMap(msg) {
    const { width, height, resolution, origin } = msg.info;
    const off = document.createElement('canvas');
    off.width = width;
    off.height = height;
    const img = off.getContext('2d').createImageData(width, height);
    for (let row = 0; row < height; row += 1) {
      // La fila 0 del OccupancyGrid es la de abajo; en la imagen va arriba
      const dst = (height - 1 - row) * width;
      for (let col = 0; col < width; col += 1) {
        const v = msg.data[row * width + col];
        const p = (dst + col) * 4;
        if (v < 0) {            // desconocido: transparente (se ve el rayado)
          img.data[p + 3] = 0;
          continue;
        }
        const t = Math.min(v, 100) / 100;
        for (let k = 0; k < 3; k += 1) {
          img.data[p + k] = COLORS.free[k] + (COLORS.wall[k] - COLORS.free[k]) * t;
        }
        img.data[p + 3] = 255;
      }
    }
    off.getContext('2d').putImageData(img, 0, 0);
    this.mapImage = off;
    this.mapInfo = {
      width, height, resolution,
      ox: origin.position.x, oy: origin.position.y,
    };
    if (!this.fitted) this.fit();
    this.dirty = true;
    return this.mapInfo;
  }

  setRobot(pose) { this.robot = pose; if (this.follow) this.centerOnRobot(); this.dirty = true; }
  setScanPoints(points) { this.scanPoints = points; this.dirty = true; }
  setShowScan(on) { this.showScan = on; this.dirty = true; }
  setPlan(points) { this.plan = points; this.dirty = true; }
  setGoal(goal) { this.goal = goal; this.dirty = true; }
  setRoute(route, progress = -1) { this.route = route; this.routeProgress = progress; this.dirty = true; }

  // Plano del mundo (lo publica web_bridge a partir del archivo .world)
  setWorld(shapes) {
    this.world = shapes;
    let b = null;
    shapes.forEach((sh) => {
      // Medio ancho y medio alto de la caja que encierra la figura
      let hx = sh.radius;
      let hy = sh.radius;
      if (sh.type !== 'cylinder') {
        const c = Math.abs(Math.cos(sh.yaw));
        const sn = Math.abs(Math.sin(sh.yaw));
        hx = (c * sh.sx + sn * sh.sy) / 2;
        hy = (sn * sh.sx + c * sh.sy) / 2;
      }
      const box = [sh.x - hx, sh.y - hy, sh.x + hx, sh.y + hy];
      b = b ? [Math.min(b[0], box[0]), Math.min(b[1], box[1]),
        Math.max(b[2], box[2]), Math.max(b[3], box[3])] : box;
    });
    this.worldBounds = b;
    if (!this.fitted) this.fit();
    this.dirty = true;
  }

  setHome(home) { this.home = home; this.dirty = true; }

  // ¿El punto cae dentro del plano (o del mapa SLAM si no hay plano)?
  contains(x, y) {
    let b = this.worldBounds;
    if (!b && this.mapInfo) {
      const m = this.mapInfo;
      b = [m.ox, m.oy, m.ox + m.width * m.resolution, m.oy + m.height * m.resolution];
    }
    return !b || (x > b[0] && x < b[2] && y > b[1] && y < b[3]);
  }

  setLayers(layers) {
    this.layers = { ...this.layers, ...layers };
    this.dirty = true;
  }

  setRotation(rad) {
    this.view.rot = rad;
    this.fit();
  }

  setFollow(on) {
    this.follow = on;
    if (on) this.centerOnRobot();
  }

  centerOnRobot() {
    if (!this.robot) return;
    this.view.cx = this.robot.x;
    this.view.cy = this.robot.y;
    this.dirty = true;
  }

  // Encuadra el plano o el mapa (o 6 m alrededor del robot si no hay nada)
  fit() {
    const w = this.canvas.clientWidth;
    const h = this.canvas.clientHeight;
    if (!w || !h) return;
    let b = null;
    if (this.worldBounds && this.layers.plano) {
      b = this.worldBounds;
    } else if (this.mapInfo) {
      const m = this.mapInfo;
      b = [m.ox, m.oy, m.ox + m.width * m.resolution, m.oy + m.height * m.resolution];
    }
    let widthM = 6;
    let heightM = 6;
    if (b) {
      widthM = b[2] - b[0];
      heightM = b[3] - b[1];
      this.view.cx = (b[0] + b[2]) / 2;
      this.view.cy = (b[1] + b[3]) / 2;
      this.fitted = true;
    } else {
      this.view.cx = this.robot ? this.robot.x : 0;
      this.view.cy = this.robot ? this.robot.y : 0;
    }
    // Tamano que ocupa en pantalla una vez girado
    const c = Math.abs(Math.cos(this.view.rot));
    const s = Math.abs(Math.sin(this.view.rot));
    const screenW = c * widthM + s * heightM;
    const screenH = s * widthM + c * heightM;
    this.view.scale = 0.92 * Math.min(w / screenW, h / screenH);
    this.dirty = true;
  }

  // ------------------------------------------------------ Coordenadas
  // Mundo -> pantalla: se gira el vector por view.rot y se invierte y
  toScreen(x, y) {
    const { cx, cy, scale, rot } = this.view;
    const dx = x - cx;
    const dy = y - cy;
    const c = Math.cos(rot);
    const s = Math.sin(rot);
    return [this.canvas.clientWidth / 2 + scale * (c * dx - s * dy),
      this.canvas.clientHeight / 2 - scale * (s * dx + c * dy)];
  }

  toWorld(px, py) {
    const { cx, cy, scale, rot } = this.view;
    const rx = (px - this.canvas.clientWidth / 2) / scale;
    const ry = -(py - this.canvas.clientHeight / 2) / scale;
    const c = Math.cos(rot);
    const s = Math.sin(rot);
    return [cx + c * rx + s * ry, cy - s * rx + c * ry];
  }

  // Angulo en pantalla (para dibujar flechas) de un rumbo del mundo
  screenAngle(yaw) {
    return yaw + this.view.rot;
  }

  // Deja el contexto dibujando en metros del mundo
  setWorldTransform() {
    const dpr = window.devicePixelRatio || 1;
    const { cx, cy, scale, rot } = this.view;
    const c = Math.cos(rot) * scale;
    const s = Math.sin(rot) * scale;
    const e = this.canvas.clientWidth / 2 - (c * cx - s * cy);
    const f = this.canvas.clientHeight / 2 + (s * cx + c * cy);
    this.ctx.setTransform(dpr * c, -dpr * s, -dpr * s, -dpr * c, dpr * e, dpr * f);
  }

  resize() {
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = Math.round(this.canvas.clientWidth * dpr);
    this.canvas.height = Math.round(this.canvas.clientHeight * dpr);
    if (!this.fitted) this.fit();
    this.dirty = true;
  }

  // ------------------------------------------------------- Interaccion
  bindPointer() {
    const c = this.canvas;
    const stage = c.parentElement;
    let pan = null;
    let press = null;

    const local = (e) => {
      const r = c.getBoundingClientRect();
      return [e.clientX - r.left, e.clientY - r.top];
    };

    c.addEventListener('contextmenu', (e) => e.preventDefault());

    c.addEventListener('pointerdown', (e) => {
      const [px, py] = local(e);
      c.setPointerCapture(e.pointerId);
      if (e.button === 2 || e.button === 1) {
        // Punto del mundo que queda "pegado" al cursor mientras se arrastra
        pan = { anchor: this.toWorld(px, py) };
        stage.classList.add('panning');
        this.follow = false;
        return;
      }
      if (e.button !== 0) return;
      const [x, y] = this.toWorld(px, py);
      press = { px, py, x, y, dragged: false };
      this.drag = { x, y, yaw: this.defaultYaw(x, y) };
      this.dirty = true;
    });

    c.addEventListener('pointermove', (e) => {
      const [px, py] = local(e);
      const [x, y] = this.toWorld(px, py);
      this.onCursor(x, y);
      if (pan) {
        this.view.cx += pan.anchor[0] - x;
        this.view.cy += pan.anchor[1] - y;
        this.dirty = true;
      } else if (press && Math.hypot(px - press.px, py - press.py) > DRAG_MIN_PX) {
        this.drag.yaw = Math.atan2(y - press.y, x - press.x);
        press.dragged = true;
        this.dirty = true;
      }
    });

    const release = () => {
      if (pan) {
        pan = null;
        stage.classList.remove('panning');
      }
      if (press) {
        const pick = { ...this.drag, dragged: press.dragged };
        press = null;
        this.drag = null;
        this.dirty = true;
        this.onPick(pick);
      }
    };
    c.addEventListener('pointerup', release);
    c.addEventListener('pointercancel', () => {
      press = null;
      pan = null;
      this.drag = null;
      stage.classList.remove('panning');
      this.dirty = true;
    });
    c.addEventListener('pointerleave', () => this.onCursor(null));

    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      const [px, py] = local(e);
      const [wx, wy] = this.toWorld(px, py);
      const factor = Math.exp(-e.deltaY * 0.0015);
      this.view.scale = Math.min(400, Math.max(8, this.view.scale * factor));
      // Mantiene fijo el punto bajo el cursor
      const [nx, ny] = this.toWorld(px, py);
      this.view.cx += wx - nx;
      this.view.cy += wy - ny;
      this.dirty = true;
    }, { passive: false });
  }

  // Sin arrastrar, la meta mira en la direccion en que llega el robot
  defaultYaw(x, y) {
    if (!this.robot) return 0;
    const d = Math.hypot(x - this.robot.x, y - this.robot.y);
    return d < 0.05 ? this.robot.yaw : Math.atan2(y - this.robot.y, x - this.robot.x);
  }

  // ------------------------------------------------------------ Dibujo
  draw() {
    this.dirty = false;
    const { ctx } = this;
    const dpr = window.devicePixelRatio || 1;
    const w = this.canvas.clientWidth;
    const h = this.canvas.clientHeight;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const showPlano = this.layers.plano && this.world;
    // Sin plano disponible se muestra el mapa SLAM aunque no este elegido
    const showSlam = this.mapImage && (this.layers.slam || !showPlano);
    if (showPlano) this.drawWorld(!showSlam);
    if (showSlam) {
      const m = this.mapInfo;
      this.setWorldTransform();
      ctx.translate(m.ox, m.oy + m.height * m.resolution);
      ctx.scale(m.resolution, -m.resolution);
      ctx.imageSmoothingEnabled = false;
      // Con las dos capas, el mapa SLAM va semitransparente sobre el plano
      ctx.globalAlpha = showPlano ? 0.55 : 1;
      ctx.drawImage(this.mapImage, 0, 0);
      ctx.globalAlpha = 1;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }
    this.drawGrid(w, h);
    this.drawPlan();
    this.drawRoute();
    this.drawScan();
    if (this.home) this.drawHome();
    if (this.goal) this.drawPose(this.goal, COLORS.robot, true);
    if (this.drag) this.drawPose(this.drag, COLORS.robot, true);
    this.drawRobot();
  }

  // Plano vectorial: piso, paredes y obstaculos con el color del .world
  drawWorld(withFloor) {
    const { ctx } = this;
    this.setWorldTransform();
    const px = 1 / this.view.scale;          // 1 pixel en metros
    if (withFloor && this.worldBounds) {
      const b = this.worldBounds;
      ctx.fillStyle = `rgb(${COLORS.free.join(',')})`;
      ctx.fillRect(b[0], b[1], b[2] - b[0], b[3] - b[1]);
    }
    ctx.lineWidth = 1.5 * px;
    ctx.strokeStyle = 'rgba(232, 239, 245, 0.9)';
    this.world.forEach((sh) => {
      // Color del modelo, aclarado hacia el tono papel del plano
      const rgb = sh.color.map((v, k) => Math.round(255 * v * 0.8 + COLORS.wall[k] * 0.2));
      ctx.fillStyle = `rgb(${rgb.join(',')})`;
      ctx.beginPath();
      if (sh.type === 'cylinder') {
        ctx.arc(sh.x, sh.y, sh.radius, 0, 2 * Math.PI);
      } else {
        const c = Math.cos(sh.yaw);
        const s = Math.sin(sh.yaw);
        [[1, 1], [1, -1], [-1, -1], [-1, 1]].forEach(([i, j], n) => {
          const lx = (i * sh.sx) / 2;
          const ly = (j * sh.sy) / 2;
          const x = sh.x + lx * c - ly * s;
          const y = sh.y + lx * s + ly * c;
          if (n === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        });
        ctx.closePath();
      }
      ctx.fill();
      ctx.stroke();
    });
    const dpr = window.devicePixelRatio || 1;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  drawGrid(w, h) {
    const { ctx } = this;
    // Zona del mundo visible (las 4 esquinas, por si la vista esta girada)
    const corners = [[0, 0], [w, 0], [0, h], [w, h]].map(([x, y]) => this.toWorld(x, y));
    const xs = corners.map((p) => p[0]);
    const ys = corners.map((p) => p[1]);
    const [xMin, xMax, yMin, yMax] = [Math.min(...xs), Math.max(...xs),
      Math.min(...ys), Math.max(...ys)];
    const step = this.view.scale > 25 ? 1 : 5;
    this.setWorldTransform();
    ctx.lineWidth = 1 / this.view.scale;
    ctx.strokeStyle = COLORS.grid;
    ctx.beginPath();
    for (let x = Math.ceil(xMin / step) * step; x <= xMax; x += step) {
      ctx.moveTo(x, yMin);
      ctx.lineTo(x, yMax);
    }
    for (let y = Math.ceil(yMin / step) * step; y <= yMax; y += step) {
      ctx.moveTo(xMin, y);
      ctx.lineTo(xMax, y);
    }
    ctx.stroke();
    // Ejes del origen: rojo = x, verde = y (como en Gazebo y RViz)
    ctx.lineWidth = 2 / this.view.scale;
    ctx.strokeStyle = 'rgba(255, 107, 107, 0.8)';
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(0.6, 0);
    ctx.stroke();
    ctx.strokeStyle = 'rgba(111, 211, 168, 0.8)';
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(0, 0.6);
    ctx.stroke();
    const dpr = window.devicePixelRatio || 1;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  // Impactos del LiDAR: puntos chicos sobre las superficies que toca
  drawScan() {
    const { ctx } = this;
    if (!this.showScan || !this.scanPoints.length) return;
    ctx.fillStyle = COLORS.scan;
    const r = Math.max(1, Math.min(2, this.view.scale * 0.02));
    ctx.beginPath();
    this.scanPoints.forEach(([x, y]) => {
      const [sx, sy] = this.toScreen(x, y);
      ctx.moveTo(sx + r, sy);
      ctx.arc(sx, sy, r, 0, 2 * Math.PI);
    });
    ctx.fill();
  }

  drawPlan() {
    const { ctx } = this;
    if (this.plan.length < 2) return;
    ctx.strokeStyle = COLORS.plan;
    ctx.lineWidth = 2;
    ctx.setLineDash([]);
    ctx.beginPath();
    this.plan.forEach(([x, y], i) => {
      const [sx, sy] = this.toScreen(x, y);
      if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
    });
    ctx.stroke();
  }

  drawRoute() {
    const { ctx } = this;
    if (!this.route.length) return;
    ctx.strokeStyle = COLORS.route;
    ctx.lineWidth = 1.5;
    ctx.setLineDash([6, 5]);
    ctx.beginPath();
    this.route.forEach((p, i) => {
      const [sx, sy] = this.toScreen(p.x, p.y);
      if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
    });
    ctx.stroke();
    ctx.setLineDash([]);
    this.route.forEach((p, i) => {
      const [sx, sy] = this.toScreen(p.x, p.y);
      const done = this.routeProgress > i;
      ctx.fillStyle = done ? '#6fd3a8' : COLORS.route;
      ctx.beginPath();
      ctx.arc(sx, sy, 10, 0, 2 * Math.PI);
      ctx.fill();
      // Marca de orientacion
      ctx.strokeStyle = ctx.fillStyle;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(sx, sy);
      const a = this.screenAngle(p.yaw);
      ctx.lineTo(sx + 17 * Math.cos(a), sy - 17 * Math.sin(a));
      ctx.stroke();
      ctx.fillStyle = COLORS.ink;
      ctx.font = '600 12px "Barlow Semi Condensed", sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(String(i + 1), sx, sy + 0.5);
    });
  }

  // Punto de inicio (spawn): anillo punteado con su orientacion
  drawHome() {
    const { ctx, home } = this;
    const [sx, sy] = this.toScreen(home.x, home.y);
    const a = this.screenAngle(home.yaw);
    ctx.strokeStyle = 'rgba(232, 239, 245, 0.75)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.arc(sx, sy, 12, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.beginPath();
    ctx.moveTo(sx, sy);
    ctx.lineTo(sx + 20 * Math.cos(a), sy - 20 * Math.sin(a));
    ctx.stroke();
  }

  // Meta: anillo con una flecha que indica la orientacion final
  drawPose(p, color, ring) {
    const { ctx } = this;
    const [sx, sy] = this.toScreen(p.x, p.y);
    const len = Math.max(26, 0.5 * this.view.scale);
    const a = this.screenAngle(p.yaw);
    const ex = sx + len * Math.cos(a);
    const ey = sy - len * Math.sin(a);
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = 2.5;
    if (ring) {
      ctx.beginPath();
      ctx.arc(sx, sy, 9, 0, 2 * Math.PI);
      ctx.stroke();
    }
    ctx.beginPath();
    ctx.moveTo(sx, sy);
    ctx.lineTo(ex, ey);
    ctx.stroke();
    const head = 9;
    ctx.beginPath();
    ctx.moveTo(ex, ey);
    ctx.lineTo(ex - head * Math.cos(a - 0.45), ey + head * Math.sin(a - 0.45));
    ctx.lineTo(ex - head * Math.cos(a + 0.45), ey + head * Math.sin(a + 0.45));
    ctx.closePath();
    ctx.fill();
  }

  drawRobot() {
    const { ctx, robot } = this;
    if (!robot) return;
    const c = Math.cos(robot.yaw);
    const s = Math.sin(robot.yaw);
    const pts = FOOTPRINT.map(([fx, fy]) => this.toScreen(
      robot.x + fx * c - fy * s, robot.y + fx * s + fy * c));
    ctx.fillStyle = 'rgba(255, 159, 67, 0.28)';
    ctx.strokeStyle = COLORS.robot;
    ctx.lineWidth = 2;
    ctx.beginPath();
    pts.forEach(([x, y], i) => (i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y)));
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    // Nariz: indica hacia donde avanza
    const [nx, ny] = this.toScreen(robot.x + 0.33 * c, robot.y + 0.33 * s);
    const [bx, by] = this.toScreen(robot.x, robot.y);
    ctx.beginPath();
    ctx.moveTo(bx, by);
    ctx.lineTo(nx, ny);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(bx, by, 3, 0, 2 * Math.PI);
    ctx.fillStyle = COLORS.robot;
    ctx.fill();
  }
}

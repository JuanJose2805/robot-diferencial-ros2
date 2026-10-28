// Copyright 2026 Juan José Martínez Medina
// Licensed under the Apache License, Version 2.0
//
// Punto de entrada: conecta la interfaz con ROS a traves de rosbridge y une
// los modulos (mapa, LiDAR, teleoperacion, navigacion y herramientas SLAM).

import { LidarView } from './lidar_view.js';
import { MapView } from './map_view.js';
import {
  RosConnection, SERVICES, TOPICS, poseMessage, yawFromQuaternion,
} from './ros.js';
import { Teleop } from './teleop.js';

const $ = (id) => document.getElementById(id);
const deg = (rad) => (rad * 180) / Math.PI;
const rad = (d) => (d * Math.PI) / 180;
const fmt = (n, d = 2) => Number(n).toFixed(d);

const state = {
  tool: 'goal',
  route: [],
  status: null,
  navState: 'idle',
  mapping: true,       // false cuando el mapeo esta en pausa
  linkRos: 'off',
  shuttingDown: false,
  lastNavKey: null,    // ultimo estado de navegacion visto (tipo:estado)
  sentRoute: null,     // la ruta que se mando a Nav2 (para limpiarla luego)
};

// -------------------------------------------------------------- Registro
function log(text, level = 'info') {
  const li = document.createElement('li');
  li.dataset.level = level;
  const time = document.createElement('time');
  time.textContent = new Date().toLocaleTimeString('es', { hour12: false });
  const span = document.createElement('span');
  span.textContent = text;
  li.append(time, span);
  const list = $('log');
  list.prepend(li);
  while (list.children.length > 60) list.lastChild.remove();
}

function setLink(id, linkState) {
  $(id).dataset.state = linkState;
}

// --------------------------------------------------------- Conexion ROS
const ros = new RosConnection({
  onState: (linkState, text) => {
    if (state.shuttingDown && linkState === 'on') {
      // Se volvio a lanzar el sistema: la pagina sigue sirviendo sin recargar
      resetShutdown();
    }
    if (state.shuttingDown && linkState !== 'on') {
      // El sistema se apago a pedido: no es un error de conexion
      $('shutdown-banner').hidden = false;
      $('btn-shutdown').textContent = 'Apagado';
      state.linkRos = 'off';
      setLink('link-ros', 'off');
      setLink('link-nav', 'off');
      setLink('link-slam', 'off');
      return;
    }
    if (linkState === state.linkRos) return;   // evita repetir el mismo aviso
    state.linkRos = linkState;
    setLink('link-ros', linkState);
    log(text, linkState === 'on' ? 'info' : 'warn');
    if (linkState !== 'on') {
      setLink('link-nav', 'off');
      setLink('link-slam', 'off');
    }
  },
});

// ------------------------------------------------------------------ Mapa
const mapView = new MapView($('map-canvas'), {
  onPick: (pose) => {
    if (state.tool === 'route') {
      // Sin arrastre, el punto mira en la direccion en que se llega desde
      // el punto anterior de la ruta
      const prev = state.route[state.route.length - 1];
      if (!pose.dragged && prev) {
        pose.yaw = Math.atan2(pose.y - prev.y, pose.x - prev.x);
      }
      addWaypoint({ x: pose.x, y: pose.y, yaw: pose.yaw });
    } else {
      fillForm(pose);
      sendGoal(pose);
    }
  },
  onCursor: (x, y) => {
    $('cursor-readout').textContent = x == null ? '' : `x ${fmt(x)}  y ${fmt(y)}`;
  },
});

document.querySelectorAll('[data-tool]').forEach((btn) => {
  btn.addEventListener('click', () => {
    state.tool = btn.dataset.tool;
    document.querySelectorAll('[data-tool]').forEach((b) => {
      b.setAttribute('aria-checked', String(b === btn));
    });
  });
});

$('btn-fit').addEventListener('click', () => mapView.fit());

// Giro de la vista: por defecto x hacia arriba, como la camara de Gazebo.
// Se recuerda en este navegador.
const ROT_KEY = 'robot_web.rotation';
let rotationDeg = 90;
try {
  const saved = localStorage.getItem(ROT_KEY);
  if (saved !== null) rotationDeg = Number(saved) % 360;
} catch (err) { /* sin almacenamiento: se usa el valor por defecto */ }
mapView.setRotation(rad(rotationDeg));
$('btn-rotate').addEventListener('click', () => {
  rotationDeg = (rotationDeg + 90) % 360;
  mapView.setRotation(rad(rotationDeg));
  try { localStorage.setItem(ROT_KEY, String(rotationDeg)); } catch (err) { /* ignorar */ }
});

// Capas: plano dibujado del mundo, mapa de SLAM o ambos superpuestos
let layerChosen = false;   // el usuario eligio a mano: no se cambia solo
function setLayer(layer) {
  mapView.setLayers({ plano: layer !== 'slam', slam: layer !== 'plano' });
  document.querySelectorAll('[data-layer]').forEach((b) => {
    b.setAttribute('aria-checked', String(b.dataset.layer === layer));
  });
}
document.querySelectorAll('[data-layer]').forEach((btn) => {
  btn.addEventListener('click', () => {
    layerChosen = true;
    setLayer(btn.dataset.layer);
  });
});

ros.subscribe(TOPICS.world, (msg) => {
  const layout = JSON.parse(msg.data);
  mapView.setWorld(layout.shapes);
  $('map-empty').hidden = true;
});
$('btn-follow').addEventListener('click', (e) => {
  const on = e.currentTarget.getAttribute('aria-pressed') !== 'true';
  e.currentTarget.setAttribute('aria-pressed', String(on));
  mapView.setFollow(on);
});

ros.subscribe(TOPICS.map, (msg) => {
  const info = mapView.setMap(msg);
  $('map-empty').hidden = true;
  $('map-size').textContent = `${fmt(info.width * info.resolution, 1)} × ${fmt(info.height * info.resolution, 1)} m`;
  $('map-res').textContent = `${fmt(info.resolution * 100, 0)} cm/celda`;
}, { throttle_rate: 1000, queue_length: 1 });

ros.subscribe(TOPICS.robotPose, (msg) => {
  const p = msg.pose;
  const pose = { x: p.position.x, y: p.position.y, yaw: yawFromQuaternion(p.orientation) };
  mapView.setRobot(pose);
  $('pose-readout').textContent = `x ${fmt(pose.x)}  y ${fmt(pose.y)}  θ ${fmt(deg(pose.yaw), 0)}°  (${msg.header.frame_id})`;
}, { throttle_rate: 100, queue_length: 1 });

ros.subscribe(TOPICS.scanPoints, (msg) => {
  mapView.setScanPoints(JSON.parse(msg.data).points);
}, { throttle_rate: 100, queue_length: 1 });

// Mostrar u ocultar los impactos del LiDAR sobre el mapa (se recuerda)
const SCAN_KEY = 'robot_web.show_scan';
let showScan = true;
try { showScan = localStorage.getItem(SCAN_KEY) !== 'false'; } catch (err) { /* por defecto */ }
function renderScanToggle() {
  $('btn-scan').setAttribute('aria-pressed', String(showScan));
  mapView.setShowScan(showScan);
}
$('btn-scan').addEventListener('click', () => {
  showScan = !showScan;
  try { localStorage.setItem(SCAN_KEY, String(showScan)); } catch (err) { /* ignorar */ }
  renderScanToggle();
});
renderScanToggle();

ros.subscribe(TOPICS.plan, (msg) => {
  mapView.setPlan(msg.poses.map((ps) => [ps.pose.position.x, ps.pose.position.y]));
}, { throttle_rate: 500, queue_length: 1 });

// ----------------------------------------------------------------- LiDAR
const lidarView = new LidarView($('lidar-canvas'));
let scanCount = 0;
let scanRateStart = performance.now();

ros.subscribe(TOPICS.scan, (msg) => {
  const nearest = lidarView.setScan(msg);
  $('lidar-closest').textContent = nearest
    ? `Obstáculo a ${fmt(nearest.distance)} m (${fmt(deg(nearest.angle), 0)}°)`
    : 'Sin obstáculos en rango';
  scanCount += 1;
  const dt = (performance.now() - scanRateStart) / 1000;
  if (dt >= 2) {
    $('lidar-rate').textContent = `${fmt(scanCount / dt, 1)} Hz`;
    scanCount = 0;
    scanRateStart = performance.now();
  }
}, { throttle_rate: 100, queue_length: 1 });

document.querySelectorAll('[data-range]').forEach((btn) => {
  btn.addEventListener('click', () => {
    lidarView.setRange(Number(btn.dataset.range));
    document.querySelectorAll('[data-range]').forEach((b) => {
      b.setAttribute('aria-checked', String(b === btn));
    });
  });
});

// --------------------------------------------------------- Teleoperacion
function navigating() {
  return ['sending', 'active'].includes(state.navState);
}

function cancelNavigation(reason) {
  ros.publish(TOPICS.cancel, {});
  if (reason) log(reason, 'warn');
}

const teleop = new Teleop({
  pad: $('pad'),
  linInput: $('lin-speed'),
  angInput: $('ang-speed'),
  onCommand: (v, w) => ros.publish(TOPICS.cmdVel, {
    linear: { x: v, y: 0, z: 0 }, angular: { x: 0, y: 0, z: w },
  }),
  onStart: () => {
    if (navigating()) cancelNavigation('Navegación cancelada: el robot pasó a control manual.');
  },
});

$('btn-stop').addEventListener('click', () => {
  teleop.stop();
  if (navigating()) cancelNavigation('Parada: navegación cancelada.');
});

$('lin-speed').addEventListener('input', (e) => {
  $('lin-out').textContent = `${fmt(e.target.value)} m/s`;
});
$('ang-speed').addEventListener('input', (e) => {
  $('ang-out').textContent = `${fmt(e.target.value)} rad/s`;
});

// ------------------------------------------------------------ Navegacion
function readForm() {
  const x = parseFloat($('goal-x').value);
  const y = parseFloat($('goal-y').value);
  const yawDeg = parseFloat($('goal-yaw').value) || 0;
  if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
  return { x, y, yaw: rad(yawDeg) };
}

function fillForm(pose) {
  $('goal-x').value = fmt(pose.x);
  $('goal-y').value = fmt(pose.y);
  $('goal-yaw').value = fmt(deg(pose.yaw), 0);
}

function sendGoal(pose) {
  if (!mapView.contains(pose.x, pose.y)) {
    log(`El punto (${fmt(pose.x)}, ${fmt(pose.y)}) está fuera de la arena: elige uno dentro del mapa.`, 'warn');
    return;
  }
  if (!ros.publish(TOPICS.goal, poseMessage(pose.x, pose.y, pose.yaw))) {
    log('Sin conexión: no se pudo enviar la meta.', 'error');
    return;
  }
  mapView.setGoal(pose);
  mapView.setRoute(state.route);
}

// Volver a la pose del spawn (posicion y orientacion iniciales)
$('btn-home').addEventListener('click', () => {
  const home = state.status && state.status.home;
  if (!home) {
    log('Todavía no se conoce el punto de inicio (esperando a web_bridge).', 'warn');
    return;
  }
  fillForm(home);
  sendGoal(home);
  log('Volviendo al punto de inicio.');
});

$('goal-form').addEventListener('submit', (e) => {
  e.preventDefault();
  const pose = readForm();
  if (pose) sendGoal(pose);
});

$('btn-add').addEventListener('click', () => {
  const pose = readForm();
  if (pose) addWaypoint(pose);
});

function addWaypoint(pose) {
  if (!mapView.contains(pose.x, pose.y)) {
    log(`El punto (${fmt(pose.x)}, ${fmt(pose.y)}) está fuera de la arena: no se agregó.`, 'warn');
    return;
  }
  // Si la lista actual es la ruta ya enviada, el punto empieza una nueva
  if (state.sentRoute && state.route === state.sentRoute) state.route = [];
  state.route.push(pose);
  renderRoute();
}

function renderRoute(progress = -1) {
  const list = $('route-list');
  list.replaceChildren(...state.route.map((p, i) => {
    const li = document.createElement('li');
    if (progress > i) li.classList.add('done');
    if (progress === i) li.classList.add('current');
    const text = document.createElement('span');
    text.className = 'coords-text';
    text.textContent = `x ${fmt(p.x)}   y ${fmt(p.y)}   θ ${fmt(deg(p.yaw), 0)}°`;
    const del = document.createElement('button');
    del.type = 'button';
    del.textContent = '✕';
    del.setAttribute('aria-label', `Quitar el punto ${i + 1}`);
    del.addEventListener('click', () => {
      state.route.splice(i, 1);
      renderRoute();
    });
    li.append(text, del);
    return li;
  }));
  $('route-empty').hidden = state.route.length > 0;
  $('route-count').textContent = state.route.length ? `(${state.route.length})` : '';
  const ready = !state.status || state.status.waypoints_ready;
  $('btn-run-route').disabled = state.route.length === 0 || !ready;
  mapView.setRoute(state.route, progress);
}

$('btn-clear-route').addEventListener('click', () => {
  state.route = [];
  renderRoute();
});

$('btn-run-route').addEventListener('click', () => {
  const msg = {
    header: { frame_id: 'map' },
    poses: state.route.map((p) => poseMessage(p.x, p.y, p.yaw)),
  };
  if (!ros.publish(TOPICS.waypoints, msg)) {
    log('Sin conexión: no se pudo enviar la ruta.', 'error');
    return;
  }
  state.sentRoute = state.route;
  mapView.setGoal(null);
});

$('btn-cancel').addEventListener('click', () => {
  cancelNavigation();
  clearNavDrawings();
});


const NAV_TITLES = {
  idle: 'En espera',
  sending: 'Enviando la meta',
  active: 'Navegando',
  succeeded: 'Meta alcanzada',
  canceled: 'Cancelada',
  aborted: 'No se pudo llegar',
  rejected: 'Meta rechazada',
  failed: 'Error de navegación',
};

function renderNav(nav) {
  state.navState = nav.state;
  $('nav-state').dataset.state = nav.state;
  $('nav-title').textContent = NAV_TITLES[nav.state] || nav.state;
  const parts = [];
  if (nav.kind === 'waypoints' && nav.total_waypoints) {
    const shown = Math.min(nav.current_waypoint + 1, nav.total_waypoints);
    parts.push(nav.state === 'succeeded'
      ? `${nav.total_waypoints} puntos recorridos`
      : `Punto ${shown} de ${nav.total_waypoints}`);
  } else if (nav.goal && !Array.isArray(nav.goal)) {
    parts.push(`Meta x ${fmt(nav.goal.x)}  y ${fmt(nav.goal.y)}`);
  }
  if (nav.distance_remaining != null && navigating()) {
    parts.push(`faltan ${fmt(nav.distance_remaining)} m`);
  }
  if (nav.recoveries) parts.push(`${nav.recoveries} recuperaciones`);
  if (nav.elapsed != null) parts.push(`${fmt(nav.elapsed, 0)} s`);
  $('nav-detail').textContent = parts.join(', ') || 'Sin meta activa';
  $('btn-cancel').disabled = !navigating();
  $('nav-readout').textContent = $('nav-title').textContent.toLowerCase();

  // El progreso de la ruta solo se pinta mientras se recorre
  const showingSent = state.sentRoute && state.route === state.sentRoute;
  if (nav.kind === 'waypoints' && navigating() && showingSent) {
    renderRoute(nav.current_waypoint);
  }
  if (!navigating() && nav.state !== 'idle') mapView.setPlan([]);

  // Acciones al CAMBIAR de estado (el estado llega 2 veces por segundo)
  const key = `${nav.kind}:${nav.state}`;
  if (key === state.lastNavKey) return;
  state.lastNavKey = key;
  if (nav.state === 'canceled' || nav.state === 'aborted') {
    clearNavDrawings();
  } else if (nav.state === 'succeeded') {
    mapView.setGoal(null);
    if (nav.kind === 'waypoints') {
      // Se muestran los puntos en verde un momento y se deja la ruta vacia
      if (showingSent) renderRoute(nav.total_waypoints);
      setTimeout(() => clearNavDrawings(), 2500);
    }
  }
}

// Limpia la meta, el camino planificado y la ruta que se envio, para poder
// armar otra. Si el usuario ya empezo a agregar puntos nuevos, se respetan.
function clearNavDrawings() {
  if (state.sentRoute && state.route === state.sentRoute) {
    state.route = [];
    state.sentRoute = null;
  }
  renderRoute();
  mapView.setGoal(null);
  mapView.setPlan([]);
}

// --------------------------------------------------------- Herramientas SLAM
$('btn-pause').addEventListener('click', async () => {
  try {
    await ros.callService(SERVICES.pauseMapping);
    state.mapping = !state.mapping;
    $('btn-pause').textContent = state.mapping ? 'Pausar el mapeo' : 'Reanudar el mapeo';
    log(state.mapping ? 'Mapeo reanudado.' : 'Mapeo en pausa: el mapa deja de actualizarse.');
  } catch (err) {
    log(`No se pudo cambiar el mapeo: ${err.message || err}`, 'error');
  }
});

$('save-form').addEventListener('submit', (e) => {
  e.preventDefault();
  if (!ros.publish(TOPICS.saveMap, { data: $('map-name').value.trim() })) {
    log('Sin conexión: no se pudo guardar el mapa.', 'error');
  }
});

function renderMapSave(save, mapsDir) {
  const hint = $('save-hint');
  hint.dataset.state = save.state;
  $('btn-save').disabled = save.state === 'saving' || !(state.status && state.status.slam_ready);
  if (save.state === 'idle') {
    hint.textContent = `Se guarda en ${mapsDir} como imagen (.pgm/.yaml) y como grafo de poses para navegar después.`;
  } else {
    hint.textContent = save.state === 'saved' ? `${save.message}: ${save.path}` : save.message;
  }
}

// ------------------------------------------------- Estado del web_bridge
ros.subscribe(TOPICS.status, (msg) => {
  const s = JSON.parse(msg.data);
  state.status = s;
  $('mode-name').textContent = s.mode;
  // Al mapear interesa ver el mapa SLAM creciendo; si no, el plano limpio
  if (!layerChosen) setLayer(s.mode === 'slam' ? 'slam' : 'plano');
  setLink('link-nav', s.nav_ready ? 'on' : 'off');
  setLink('link-slam', s.slam_ready ? 'on' : 'off');
  $('btn-pause').hidden = s.mode !== 'slam' || !s.slam_ready;
  // Guardar solo tiene sentido mientras se construye un mapa nuevo
  $('save-form').hidden = s.mode !== 'slam';
  $('btn-go').disabled = !s.nav_ready;
  $('btn-home').disabled = !s.nav_ready;
  if (s.home) mapView.setHome(s.home);
  renderNav(s.nav);
  $('btn-run-route').disabled = !s.waypoints_ready || state.route.length === 0;
  renderMapSave(s.map_save, s.maps_dir);
});

ros.subscribe(TOPICS.events, (msg) => {
  const ev = JSON.parse(msg.data);
  log(ev.text, ev.level);
});

// ------------------------------------------------- Apagar todo el sistema
function resetShutdown() {
  state.shuttingDown = false;
  $('shutdown-banner').hidden = true;
  const btn = $('btn-shutdown');
  btn.disabled = false;
  btn.classList.remove('confirm');
  btn.textContent = 'Apagar sistema';
}

$('btn-banner-close').addEventListener('click', () => {
  $('shutdown-banner').hidden = true;
});

// Pide confirmacion con un segundo clic (sin ventanas emergentes)
let confirmTimer = null;
$('btn-shutdown').addEventListener('click', () => {
  const btn = $('btn-shutdown');
  if (!btn.classList.contains('confirm')) {
    btn.classList.add('confirm');
    btn.textContent = 'Confirmar apagado';
    confirmTimer = setTimeout(() => {
      btn.classList.remove('confirm');
      btn.textContent = 'Apagar sistema';
    }, 4000);
    return;
  }
  clearTimeout(confirmTimer);
  teleop.stop();
  if (!ros.publish(TOPICS.shutdown, {})) {
    log('Sin conexión: no se pudo apagar el sistema.', 'error');
    return;
  }
  state.shuttingDown = true;
  btn.disabled = true;
  btn.textContent = 'Apagando...';
  log('Apagando todo el sistema...', 'warn');
});

renderRoute();
log('Interfaz lista. Conectando con rosbridge...');
ros.connect();

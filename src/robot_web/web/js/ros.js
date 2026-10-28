// Copyright 2026 Juan José Martínez Medina
// Licensed under the Apache License, Version 2.0
//
// Conexion con rosbridge (roslibjs) y nombres de todos los topicos que usa la
// interfaz. La conexion se reintenta sola si rosbridge todavia no arranco o se
// reinicia la simulacion.

/* global ROSLIB */

// Topicos del robot (nombres absolutos, fuera del namespace de la web)
export const TOPICS = {
  cmdVel: { name: '/cmd_vel', type: 'geometry_msgs/Twist' },
  scan: { name: '/scan', type: 'sensor_msgs/LaserScan' },
  map: { name: '/map', type: 'nav_msgs/OccupancyGrid' },
  plan: { name: '/plan', type: 'nav_msgs/Path' },
  // Interfaz del nodo web_bridge, dentro del namespace /web
  robotPose: { name: '/web/robot_pose', type: 'geometry_msgs/PoseStamped' },
  scanPoints: { name: '/web/scan_points', type: 'std_msgs/String' },
  status: { name: '/web/status', type: 'std_msgs/String' },
  events: { name: '/web/events', type: 'std_msgs/String' },
  goal: { name: '/web/goal', type: 'geometry_msgs/PoseStamped' },
  waypoints: { name: '/web/waypoints', type: 'nav_msgs/Path' },
  cancel: { name: '/web/cancel', type: 'std_msgs/Empty' },
  saveMap: { name: '/web/save_map', type: 'std_msgs/String' },
  world: { name: '/web/world_layout', type: 'std_msgs/String' },
  shutdown: { name: '/web/shutdown', type: 'std_msgs/Empty' },
};

export const SERVICES = {
  pauseMapping: {
    name: '/slam_toolbox/pause_new_measurements',
    type: 'slam_toolbox/Pause',
  },
};

// Direccion de rosbridge: el mismo equipo que sirve la pagina, puerto 9090.
// Se puede cambiar con ?ws=ws://otra-ip:9090 en la URL.
function rosbridgeUrl() {
  const fromQuery = new URLSearchParams(window.location.search).get('ws');
  if (fromQuery) return fromQuery;
  const host = window.location.hostname || 'localhost';
  return `ws://${host}:9090`;
}

export class RosConnection {
  constructor({ onState }) {
    this.url = rosbridgeUrl();
    this.onState = onState;
    this.ros = new ROSLIB.Ros();
    this.connected = false;
    this.subscribers = [];   // se vuelven a crear al reconectar
    this.publishers = {};

    this.ros.on('connection', () => {
      this.connected = true;
      this.onState('on', `Conectado a rosbridge (${this.url})`);
      this.subscribers.forEach((s) => s.attach());
    });
    this.ros.on('error', () => {
      this.onState('error', `No se pudo conectar a ${this.url}`);
    });
    this.ros.on('close', () => {
      const was = this.connected;
      this.connected = false;
      if (was) this.onState('off', 'Se perdió la conexión con rosbridge');
      this.subscribers.forEach((s) => s.detach());
      this.publishers = {};
      setTimeout(() => this.connect(), 2000);
    });
  }

  connect() {
    this.ros.connect(this.url);
  }

  // Suscripcion que sobrevive a las reconexiones
  subscribe(topic, callback, options = {}) {
    const entry = {
      handle: null,
      attach: () => {
        entry.handle = new ROSLIB.Topic({
          ros: this.ros, name: topic.name, messageType: topic.type, ...options,
        });
        entry.handle.subscribe(callback);
      },
      detach: () => {
        if (entry.handle) entry.handle.unsubscribe();
        entry.handle = null;
      },
    };
    this.subscribers.push(entry);
    if (this.connected) entry.attach();
  }

  publish(topic, message) {
    if (!this.connected) return false;
    if (!this.publishers[topic.name]) {
      this.publishers[topic.name] = new ROSLIB.Topic({
        ros: this.ros, name: topic.name, messageType: topic.type,
      });
    }
    this.publishers[topic.name].publish(new ROSLIB.Message(message));
    return true;
  }

  callService(service, request = {}) {
    return new Promise((resolve, reject) => {
      if (!this.connected) {
        reject(new Error('Sin conexión con rosbridge'));
        return;
      }
      const client = new ROSLIB.Service({
        ros: this.ros, name: service.name, serviceType: service.type,
      });
      client.callService(new ROSLIB.ServiceRequest(request), resolve, reject);
    });
  }
}

// Utilidades de geometria compartidas
export function yawFromQuaternion(q) {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

export function poseMessage(x, y, yaw, frame = 'map') {
  return {
    header: { frame_id: frame },
    pose: {
      position: { x, y, z: 0 },
      orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) },
    },
  };
}

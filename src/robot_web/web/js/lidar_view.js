// Copyright 2026 Juan José Martínez Medina
// Licensed under the Apache License, Version 2.0
//
// Vista polar del LiDAR centrada en el robot (el frente apunta hacia arriba).
// Muestra cada lectura de /scan y el obstaculo mas cercano.

const NEAR = [255, 159, 67];   // naranja: cerca
const FAR = [143, 171, 196];   // azul grisaceo: lejos

export class LidarView {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.range = 5;
    this.scan = null;
    const dpr = window.devicePixelRatio || 1;
    canvas.width = 220 * dpr;
    canvas.height = 220 * dpr;
    this.draw();
  }

  setRange(meters) {
    this.range = meters;
    this.draw();
  }

  // Devuelve {distance, angle} del punto mas cercano (o null)
  setScan(scan) {
    this.scan = scan;
    this.draw();
    let best = null;
    scan.ranges.forEach((r, i) => {
      if (r > scan.range_min && r < scan.range_max && (!best || r < best.distance)) {
        best = { distance: r, angle: scan.angle_min + i * scan.angle_increment };
      }
    });
    return best;
  }

  draw() {
    const { ctx, canvas } = this;
    const dpr = window.devicePixelRatio || 1;
    const size = canvas.width / dpr;
    const c = size / 2;
    const k = (c - 6) / this.range;      // px por metro
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, size, size);

    // Anillos de distancia
    ctx.strokeStyle = 'rgba(143, 171, 196, 0.22)';
    ctx.fillStyle = 'rgba(143, 171, 196, 0.7)';
    ctx.font = '11px Barlow, sans-serif';
    ctx.lineWidth = 1;
    const step = this.range <= 2 ? 0.5 : this.range <= 5 ? 1 : 2.5;
    for (let r = step; r <= this.range + 1e-6; r += step) {
      ctx.beginPath();
      ctx.arc(c, c, r * k, 0, 2 * Math.PI);
      ctx.stroke();
      ctx.fillText(`${r} m`, c + 3, c - r * k + 12);
    }
    ctx.beginPath();
    ctx.moveTo(c, 4);
    ctx.lineTo(c, size - 4);
    ctx.moveTo(4, c);
    ctx.lineTo(size - 4, c);
    ctx.stroke();

    // Lecturas: angulo 0 = frente del robot = arriba en la vista
    const { scan } = this;
    if (scan) {
      for (let i = 0; i < scan.ranges.length; i += 1) {
        const r = scan.ranges[i];
        if (!(r > scan.range_min && r < scan.range_max) || r > this.range) continue;
        const a = scan.angle_min + i * scan.angle_increment;
        const x = c - r * k * Math.sin(a);
        const y = c - r * k * Math.cos(a);
        const t = Math.min(1, r / this.range);
        ctx.fillStyle = `rgb(${NEAR.map((n, j) => Math.round(n + (FAR[j] - n) * t)).join(',')})`;
        ctx.fillRect(x - 1.5, y - 1.5, 3, 3);
      }
    }

    // El robot, a escala aproximada, mirando hacia arriba
    ctx.fillStyle = '#ff9f43';
    ctx.beginPath();
    ctx.moveTo(c, c - 9);
    ctx.lineTo(c + 6, c + 6);
    ctx.lineTo(c - 6, c + 6);
    ctx.closePath();
    ctx.fill();
  }
}

// Copyright 2026 Juan José Martínez Medina
// Licensed under the Apache License, Version 2.0
//
// Teleoperacion desde la web: botones (raton o tactil) y teclado.
// Mientras haya una direccion activa publica /cmd_vel a 10 Hz; al soltar
// envia velocidad cero para que el robot se detenga en el acto.

const KEYS = {
  KeyW: 'fwd', ArrowUp: 'fwd',
  KeyS: 'back', ArrowDown: 'back',
  KeyA: 'left', ArrowLeft: 'left',
  KeyD: 'right', ArrowRight: 'right',
};

export class Teleop {
  constructor({ pad, linInput, angInput, onCommand, onStart }) {
    this.pad = pad;
    this.linInput = linInput;
    this.angInput = angInput;
    this.onCommand = onCommand;   // (linear, angular) => void
    this.onStart = onStart;       // se llama al empezar a mover
    this.active = new Set();
    this.timer = null;

    pad.querySelectorAll('[data-dir]').forEach((btn) => {
      const dir = btn.dataset.dir;
      btn.addEventListener('pointerdown', (e) => {
        btn.setPointerCapture(e.pointerId);
        this.press(dir);
      });
      const up = () => this.release(dir);
      btn.addEventListener('pointerup', up);
      btn.addEventListener('pointercancel', up);
      btn.addEventListener('lostpointercapture', up);
    });

    window.addEventListener('keydown', (e) => {
      if (this.isTyping(e)) return;
      if (e.code === 'Space') {
        e.preventDefault();
        this.stop();
        return;
      }
      const dir = KEYS[e.code];
      if (!dir) return;
      e.preventDefault();
      if (!e.repeat) this.press(dir);
    });
    window.addEventListener('keyup', (e) => {
      const dir = KEYS[e.code];
      if (dir) this.release(dir);
    });
    // Si la ventana pierde el foco con una tecla apretada, frena
    window.addEventListener('blur', () => {
      if (this.active.size) this.stop();
    });
  }

  isTyping(e) {
    const t = e.target;
    return t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.isContentEditable);
  }

  press(dir) {
    if (this.active.has(dir)) return;
    if (this.active.size === 0 && this.onStart) this.onStart();
    this.active.add(dir);
    this.render();
    this.send();
    if (!this.timer) this.timer = setInterval(() => this.send(), 100);
  }

  release(dir) {
    if (!this.active.delete(dir)) return;
    this.render();
    if (this.active.size === 0) this.halt();
    else this.send();
  }

  stop() {
    this.active.clear();
    this.render();
    this.halt();
  }

  halt() {
    clearInterval(this.timer);
    this.timer = null;
    this.onCommand(0, 0);
  }

  send() {
    const lin = parseFloat(this.linInput.value);
    const ang = parseFloat(this.angInput.value);
    let v = 0;
    let w = 0;
    if (this.active.has('fwd')) v += lin;
    if (this.active.has('back')) v -= lin;
    if (this.active.has('left')) w += ang;
    if (this.active.has('right')) w -= ang;
    this.onCommand(v, w);
  }

  render() {
    this.pad.querySelectorAll('[data-dir]').forEach((btn) => {
      btn.classList.toggle('active', this.active.has(btn.dataset.dir));
    });
  }
}

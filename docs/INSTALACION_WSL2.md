# Instalar Ubuntu 20.04 (WSL2) y ROS 2 Foxy en Windows

Guía para preparar un equipo con **Windows 10/11** desde cero hasta tener
ROS 2 Foxy funcionando. Después de esto se sigue el
[README](../README.md#instalación) para instalar y compilar el proyecto.

ROS 2 Foxy solo tiene paquetes para **Ubuntu 20.04 (focal)**. Por eso se usa
esa versión de Ubuntu, aunque haya otras más nuevas.

**Contenido**
1. Requisitos de Windows
2. Instalar Ubuntu 20.04 en WSL2
3. Primer arranque y usuario
4. Preparar Ubuntu
5. Instalar ROS 2 Foxy
6. Comprobar que todo funciona
7. Consejos para trabajar en WSL2
8. Problemas frecuentes

---

## 1. Requisitos de Windows

- **Sistema:**
  - Windows 11 (recomendado, trae WSLg para abrir ventanas gráficas como Gazebo y RViz);
  - o Windows 10 versión 21H2 o superior.
- **Espacio y memoria:** unos 15 GB libres en disco; 8 GB de RAM como mínimo (16 GB recomendados para Gazebo + Nav2).
- **Virtualización activada en la BIOS/UEFI.** Se comprueba en *Administrador de tareas → Rendimiento → CPU → Virtualización: Habilitado*.
  - Si dice "Deshabilitado", hay que activar Intel VT-x o AMD-V (SVM) en la BIOS. Cómo hacerlo depende del fabricante de la placa.
- **Controladores de la tarjeta gráfica actualizados** (NVIDIA, AMD o Intel). WSLg los usa para acelerar Gazebo y RViz.

## 2. Instalar Ubuntu 20.04 en WSL2

Abre **PowerShell como administrador** (clic derecho en Inicio → Terminal (Administrador)).

### 2.1 Opción rápida

```powershell
wsl --install -d Ubuntu-20.04
```

Instala WSL (si hace falta) y Ubuntu 20.04. Si pide reiniciar, reinicia, y al
volver se abre Ubuntu solo. Sigue en la [sección 3](#3-primer-arranque-y-usuario).

Para ver qué distribuciones ofrece tu Windows:

```powershell
wsl --list --online
```

### 2.2 Opción manual (si `Ubuntu-20.04` no aparece en la lista)

En algunos equipos la lista no incluye Ubuntu 20.04. En ese caso:

1. **Activa los dos componentes de Windows** que necesita WSL2, uno por vez:
   ```powershell
   dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
   ```
   ```powershell
   dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
   ```
2. **Reinicia Windows.**
3. **Haz que las distribuciones nuevas usen WSL2:**
   ```powershell
   wsl --set-default-version 2
   ```
4. **Descarga Ubuntu 20.04 LTS (x64)** desde la
   [página de descarga manual de Microsoft](https://learn.microsoft.com/windows/wsl/install-manual#downloading-distributions)
   o desde la Microsoft Store, e instálalo. Al terminar, ábrelo con **Iniciar**.

> Si al abrir Ubuntu aparece
> `WslRegisterDistribution failed with error: 0x80370114`, falta alguno de los
> componentes del paso 1 o la virtualización está apagada en la BIOS. Repite
> el paso 1, reinicia y revisa la virtualización (sección 1).

### 2.3 Confirmar que es WSL2

```powershell
wsl --list --verbose
```

En la columna **VERSION** de `Ubuntu-20.04` debe decir **2**. Si dice 1:

```powershell
wsl --set-version Ubuntu-20.04 2
```

## 3. Primer arranque y usuario

La primera vez, Ubuntu pide crear un **usuario y contraseña de Linux**. No
tienen que coincidir con los de Windows. Al escribir la contraseña no aparecen
caracteres en pantalla; es normal.

Comprueba que entraste con tu usuario y no como `root`:

```bash
whoami
```

Debe mostrar tu usuario. Si la terminal muestra `root@...` (pasa si la ventana
se cerró durante la configuración inicial), crea el usuario a mano. En los
comandos, reemplaza `<usuario>` por el nombre que quieras:

```bash
adduser <usuario>
usermod -aG sudo <usuario>
printf '[user]\ndefault=<usuario>\n' > /etc/wsl.conf
```

Después, desde **PowerShell**, reinicia WSL y vuelve a abrir Ubuntu:

```powershell
wsl --shutdown
```

Comprueba la versión de Ubuntu:

```bash
lsb_release -a
```

Debe decir `Ubuntu 20.04.x LTS` y `Codename: focal`.

> **No ejecutes `do-release-upgrade`.** Si Ubuntu sube a 22.04, ROS 2 Foxy deja de poder instalarse.

## 4. Preparar Ubuntu

Actualiza el sistema e instala herramientas básicas:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl git wget gnupg2 lsb-release software-properties-common build-essential python3-pip
```

ROS 2 necesita una configuración regional con UTF-8:

```bash
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
```

En la salida de `locale` deben aparecer valores con `UTF-8`. Puede salir alguna
advertencia de Perl (`Setting locale failed`) la primera vez; desaparece al
abrir una terminal nueva.

## 5. Instalar ROS 2 Foxy

Se sigue la
[guía oficial de Foxy con paquetes Debian](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).
Se usa `apt` porque es mucho más rápido que compilar ROS desde el código fuente.

### 5.1 Agregar el repositorio de ROS 2

```bash
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
```

Qué hace cada línea:
1. Activa el repositorio `universe` de Ubuntu, donde están varias dependencias.
2. Descarga la clave con la que están firmados los paquetes de ROS 2.
3. Registra el repositorio de ROS 2 para Ubuntu 20.04.
4. Actualiza la lista de paquetes.

### 5.2 Instalar ROS 2 Foxy Desktop

```bash
sudo apt install -y ros-foxy-desktop python3-argcomplete ros-dev-tools
```

- `ros-foxy-desktop`: ROS 2 completo con RViz2, demos y herramientas.
- `python3-argcomplete`: autocompletado de los comandos `ros2` con Tab.
- `ros-dev-tools`: herramientas de desarrollo (`colcon`, `rosdep`, `vcstool`).

### 5.3 Cargar ROS 2 en cada terminal

Para no tener que escribir `source` cada vez que abres una terminal:

```bash
grep -qxF 'source /opt/ros/foxy/setup.bash' ~/.bashrc || echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
source ~/.bashrc
echo $ROS_DISTRO
```

Debe mostrar `foxy`. El `grep` evita agregar la línea dos veces si ya estaba.

## 6. Comprobar que todo funciona

**Comunicación entre nodos.** Abre dos terminales de Ubuntu.

Terminal 1, publicador:
```bash
ros2 run demo_nodes_cpp talker
```

Terminal 2, suscriptor:
```bash
ros2 run demo_nodes_py listener
```

La terminal 2 debe mostrar `I heard: [Hello World: 1]`, `2`, `3`… Cierra ambas con **Ctrl+C**.

**Ventanas gráficas (WSLg).**
```bash
rviz2
```

Debe abrirse la ventana de RViz2 en Windows. Si se abre, Gazebo también podrá.

Con esto el equipo está listo. Continúa con la
[instalación del proyecto en el README](../README.md#instalación), desde el
paso 2 (dependencias del proyecto).

## 7. Consejos para trabajar en WSL2

- **Trabaja dentro de Linux** (`~/...`), no en `/mnt/c/...`. Los archivos de Windows se leen mucho más lento desde WSL, y `colcon build` se nota.
- **Para pasar archivos entre Windows y Ubuntu,** Ubuntu ve el disco de Windows en `/mnt/c/` (por ejemplo, Descargas está en `/mnt/c/Users/<usuario_windows>/Downloads/`). Desde Windows, Ubuntu aparece en el Explorador como `\\wsl$\Ubuntu-20.04`.
- **El navegador de Windows llega a Ubuntu por `localhost`.** Por eso la interfaz web del proyecto se abre en `http://localhost:8000` sin configurar nada.
- **Memoria.** Por defecto WSL2 puede usar hasta la mitad de la RAM. Si Gazebo y Nav2 van lentos, se puede ajustar en `C:\Users\<usuario_windows>\.wslconfig`:
  ```ini
  [wsl2]
  memory=10GB
  processors=6
  ```
  Luego `wsl --shutdown` en PowerShell para aplicarlo.
- **Para apagar Ubuntu por completo,** cuando se queda algo colgado, usa `wsl --shutdown` en PowerShell.

## 8. Problemas frecuentes

| Problema | Solución |
|---|---|
| `0x80370114` al abrir Ubuntu | Faltan componentes de Windows o la virtualización está apagada. Ver la sección 2.2, paso 1, y la sección 1 |
| `0x80370102` | La virtualización está desactivada en la BIOS/UEFI |
| Ubuntu abre como `root` | Crear el usuario a mano (sección 3) |
| `Unable to locate package ros-foxy-desktop` | No se agregó bien el repositorio (repetir 5.1) o Ubuntu no es 20.04 (`lsb_release -a`) |
| Error de firma o clave GPG en `apt update` | Volver a descargar la clave (segunda línea de 5.1) y repetir `sudo apt update` |
| `ros2: command not found` | Falta cargar ROS: `source /opt/ros/foxy/setup.bash`, o revisar la sección 5.3 |
| RViz2 o Gazebo no abren ventana | Actualizar Windows (WSLg) y los controladores de la tarjeta gráfica; ejecutar `wsl --update` en PowerShell |
| Gazebo muy lento | Ajustar la memoria en `.wslconfig` (sección 7) y cerrar programas pesados en Windows |

> ROS 2 Foxy y Gazebo Classic ya no tienen soporte oficial, pero sus paquetes
> siguen publicados en el repositorio de ROS. Si en el futuro aparece algún
> problema con un paquete antiguo, revisar la
> [documentación de Foxy](https://docs.ros.org/en/foxy/).

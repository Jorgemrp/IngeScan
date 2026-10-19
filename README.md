# IngeScan v1.0.0 — Nubes de Puntos y Escaneos 3D para IngeTrazo

**Autor:** Jorge Rodríguez Pérez  
**Versión:** 1.0.0  
**Licencia:** GPL-3.0-or-later  
**Compatibilidad:** IngeTrazo v0.5.3 – v0.5.8+ (Windows, macOS y Linux) · Cero dependencias externas

---

## Descripción

**IngeScan** (inspirado en *Trimble Scan Essentials* para SketchUp) permite importar, alinear, seccionar e inspeccionar **nubes de puntos y escaneos LiDAR 3D** directamente dentro del visor de **IngeTrazo**, utilizándolos como plantilla de referencia para medir y modelar arquitectura e ingeniería (*Scan-to-BIM*) con **enganche magnético (Snap)** a los puntos reales.

Funciona mediante un **único archivo Python autónomo (`ingescan.py`)**, sin necesidad de instalar librerías adicionales (`pip`).

---

## Características principales (v1.0)

1. **Soporte nativo multiformato (Cero dependencias externas):**
   - **`.dae` (COLLADA):** Importa mallas y escaneos LiDAR (p. ej. *3D Scanner App* / iPhone / iPad Pro) muestreando el color real RGB directamente desde la textura `.jpg` / `.png` asociada.
   - **`.ply`:** Soporte vectorizado para archivos PLY tanto en formato *ASCII* como *Binary Little Endian*.
   - **`.xyz` / `.pts` / `.txt` / `.csv`:** Nubes de puntos en texto plano separadas por espacios, comas o punto y coma.
   - **`.las`:** Lector binario nativo para archivos LiDAR estándar ASPRS LAS (1.2 – 1.4).
   - **`.e57`:** Decodificador nativo `BitPackCodec` para archivos estándar ASTM E57.
   - **Conversión automática de ejes (`Y-up` $\rightarrow$ `Z-up`):** Todos los formatos procedentes de escáneres móviles entran automáticamente nivelados en el sistema de coordenadas `Z-up` de IngeTrazo.

2. **Modo Plantilla Inerte (`🔒 Bloqueado`) vs. Edición Libre (`🔓 Desbloqueado`):**
   - Al importar, el escaneo queda fijado como **plantilla inerte** para poder orbitar, medir y dibujar encima con las herramientas de Línea, Rectángulo, Empujar/Tirar, etc., sin seleccionarlo ni desplazarlo por error.
   - Al pulsar **`🔓 Desbloquear`**, la caja envolvente del escaneo se puede seleccionar haciendo clic en el visor 3D y transformar en tiempo real con las herramientas nativas **Mover (`M`)** y **Rotar (`Q`)**, o con los botones rápidos del panel (`Centrar en X=0, Y=0`, `Apoyar suelo en Z=0`, giros de $\pm 90^\circ$ y $180^\circ$ en $Z$, $X$ e $Y$).

3. **Enganche Magnético Inteligente (*Point Cloud Snap*):**
   - El cursor de cualquier herramienta de dibujo o medición de IngeTrazo detecta y se engancha magnéticamente al punto visible más cercano de la nube.

4. **Oclusión en Profundidad tras Caras Sólidas (*Hardware Depth Occlusion*):**
   - Sincronizado con el *Z-Buffer* OpenGL del visor: los puntos situados detrás o en el interior de muros, losas y volúmenes sólidos modelados se ocultan automáticamente a 60 FPS.

5. **🌡️ Mapa de Inspección y Análisis de Desviaciones (*As-Built vs. Modelo*):**
   - Evalúa la cercanía entre las caras sólidas modeladas y los puntos reales del escaneo, tiñendo la nube con un gradiente continuo en tiempo real:
     - **🟢 Verde (`0 cm`):** El punto coincide con la cara modelada.
     - **🟡 Amarillo (`50 %` de la tolerancia):** Desviación media.
     - **🔴 Rojo (`100 %` de la tolerancia):** Fuera de tolerancia (desplomes, abombamientos o diferencias constructivas).
   - Incluye **barra deslizante de tolerancia ajustable en vivo (`1 cm` a `100 cm`)** y opción para ocultar los puntos fuera del rango de análisis.

6. **Sección Horizontal de Planta (*Clipping Z*):**
   - Acota la visualización entre `Z Mín` y `Z Máx` para aislar plantas o rebanadas horizontales y trazar tabiquería en planta con total limpieza.

---

## Instalación en IngeTrazo

1. Abre **IngeTrazo**.
2. En la barra superior, ve a **Extensiones ▸ Abrir carpeta de plugins** (*Extensions ▸ Open plugins folder*).
3. Copia el archivo [`ingescan.py`](./ingescan.py) dentro de esa carpeta:
   - **Windows:** `%APPDATA%\ingetrazo\plugins\ingescan.py`
   - **macOS / Linux:** `~/.local/share/ingetrazo/plugins/ingescan.py`
4. Reinicia **IngeTrazo**. La pestaña **IngeScan** aparecerá automáticamente en el panel lateral derecho.

---

## Créditos y Licencia

- **Autor:** Jorge Rodríguez Pérez (2026)
- **Licencia:** GNU General Public License v3.0 or later (`GPL-3.0-or-later`)

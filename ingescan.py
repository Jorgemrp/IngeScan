# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Jorge Rodríguez Pérez.
"""IngeScan v1.0.0 — Plugin de Nubes de Puntos y Escaneos 3D para IngeTrazo.

Autor: Jorge Rodríguez Pérez
Versión: 1.0.0
Licencia: GPL-3.0-or-later
Compatibilidad: IngeTrazo v0.5.3 – v0.5.8+ (Windows, macOS, Linux)

Sigue estrictamente la guía oficial de desarrollo de extensiones de IngeTrazo:
https://github.com/ingelibre/ingetrazo/blob/main/docs/plugins.md

Inspirado en Trimble Scan Essentials (SketchUp).
Permite importar escaneos LiDAR y nubes de puntos (COLLADA .dae con textura JPG,
PLY ASCII/binario, XYZ/PTS/TXT/CSV, LAS nativo y ASTM E57) como plantillas inertes
de referencia para medir y modelar directamente encima con Snap magnético.
Incluye:
- Bloqueo como plantilla inerte o desbloqueo para selección, Mover (M) y Rotar (Q).
- Alineación rápida al origen, nivelación de cota Z=0 y giros ortogonales en X, Y, Z.
- Oclusión inteligente en tiempo real detrás de caras y volúmenes sólidos modelados.
- Mapa de Inspección (As-Built vs. Modelo) con gradiente Verde -> Amarillo -> Rojo
  según barra deslizante de tolerancia ajustable en vivo (1 cm a 100 cm).
- Recorte horizontal de planta (Sección Z).
"""
from __future__ import annotations

__author__ = "Jorge Rodríguez Pérez"
__version__ = "1.0.0"
__license__ = "GPL-3.0-or-later"

import math
import os
import struct
import sys
import time
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

from PySide6.QtCore import Qt, QPointF, QRectF, Signal, QObject
from PySide6.QtGui import (
    QColor,
    QFont,
    QImage,
    QPainter,
    QPen,
    QVector3D,
    QVector4D,
    QMatrix4x4,
)
from PySide6.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLabel,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QSlider,
    QComboBox,
    QCheckBox,
    QDoubleSpinBox,
    QFileDialog,
    QMessageBox,
    QGroupBox,
    QDialog,
    QFrame,
    QGridLayout,
)

from core.group import Group
from core.mesh import Mesh
from core.history import rotation_matrix
from tools.base import Tool, ToolContext
from core.snap import SnapResult

DEBUG_LOG = os.path.join(tempfile.gettempdir(), "ingescan_debug.log")


def _debug_log(msg: str) -> None:
    try:
        with open(DEBUG_LOG, "a", encoding="utf-8") as f:
            f.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}\n")
    except Exception:
        pass


# ==============================================================================
# 1. LECTORES DE ARCHIVOS DE ESCANEO 3D (COLLADA .dae, PLY, XYZ, LAS, E57)
# ==============================================================================

_NS = "{http://www.collada.org/2005/11/COLLADASchema}"


def _sanitize_colors(colors: np.ndarray | None) -> np.ndarray | None:
    """Descarta arrays de color vacíos o donde todos los puntos tienen RGB = (0, 0, 0)."""
    if colors is None or len(colors) == 0:
        return None
    if np.max(colors) == 0:
        return None
    return colors


def _convert_yup_to_zup(coords: np.ndarray) -> np.ndarray:
    """Convierte coordenadas de sistema Y-up (ARKit / 3D Scanner App / OpenGL: X=derecha, Y=arriba, Z=profundidad)
    al sistema Z-up de IngeTrazo / CAD (X=derecha, Y=profundidad, Z=arriba).
    Equivale a un giro de +90° en X: (X, Y, Z) -> (X, -Z, Y).
    """
    if len(coords) == 0:
        return coords
    return np.column_stack((coords[:, 0], -coords[:, 2], coords[:, 1])).astype(np.float32)


def read_file(path: str) -> tuple[np.ndarray, np.ndarray | None]:
    ext = os.path.splitext(path)[1].lower()
    if ext == ".dae":
        return read_dae(path)
    elif ext == ".ply":
        coords, colors = read_ply(path)
        return _convert_yup_to_zup(coords), colors
    elif ext in (".xyz", ".pts", ".txt", ".csv"):
        coords, colors = read_xyz(path)
        return _convert_yup_to_zup(coords), colors
    elif ext in (".las", ".laz"):
        coords, colors = read_las(path)
        return _convert_yup_to_zup(coords), colors
    elif ext == ".e57":
        coords, colors = read_e57(path)
        return _convert_yup_to_zup(coords), colors
    else:
        raise ValueError(
            f"Formato no soportado: '{ext}'. Formatos válidos: .dae, .ply, .xyz, .pts, .las, .e57"
        )


def read_dae(path: str) -> tuple[np.ndarray, np.ndarray | None]:
    """Lee un archivo COLLADA (.dae) generado por escáneres (ej. iPhone LiDAR + 3D Scanner App).
    Extrae los vértices, muestrea el color real desde la textura JPG asociada (si existe) y
    convierte automáticamente el eje a Z-up de IngeTrazo.
    """
    tree = ET.parse(path)
    root = tree.getroot()
    base_dir = os.path.dirname(path)

    scale = 1.0
    unit_el = root.find(f".//{_NS}unit")
    if unit_el is not None and "meter" in unit_el.attrib:
        try:
            scale = float(unit_el.attrib["meter"])
        except ValueError:
            pass

    up_axis = "Z_UP"
    up_el = root.find(f".//{_NS}up_axis")
    if up_el is not None and up_el.text:
        up_axis = up_el.text.strip().upper()

    # Intentar cargar imagen de textura asociada (.jpg / .png)
    tex_arr = None
    for init_el in root.findall(f".//{_NS}image/{_NS}init_from"):
        if init_el.text:
            candidate = os.path.join(base_dir, init_el.text.strip())
            if os.path.isfile(candidate):
                qimg = QImage(candidate)
                if not qimg.isNull():
                    qimg = qimg.convertToFormat(QImage.Format_RGB888)
                    tw, th = qimg.width(), qimg.height()
                    bpl = qimg.bytesPerLine()
                    ptr = qimg.constBits()
                    raw = np.frombuffer(ptr, dtype=np.uint8).reshape((th, bpl))
                    tex_arr = raw[:, : tw * 3].reshape((th, tw, 3)).copy()
                    break

    coords_list = []
    colors_list = []

    for mesh_el in root.findall(f".//{_NS}mesh"):
        pos_source_id = None
        vertices_el = mesh_el.find(f"{_NS}vertices")
        if vertices_el is not None:
            for inp in vertices_el.findall(f"{_NS}input"):
                if inp.attrib.get("semantic") == "POSITION":
                    pos_source_id = inp.attrib.get("source", "").lstrip("#")
                    break

        mesh_coords = None
        if pos_source_id:
            src = mesh_el.find(f".//*[@id='{pos_source_id}']")
            if src is not None:
                fa = src.find(f"{_NS}float_array")
                if fa is not None and fa.text:
                    vals = np.fromstring(fa.text, dtype=np.float32, sep=" ")
                    if len(vals) >= 3:
                        mesh_coords = vals.reshape(-1, 3)

        if mesh_coords is None:
            continue

        mesh_colors = None
        # Extraer colores de textura mediante coordenadas UV si hay imagen
        if tex_arr is not None:
            uv_source_id = None
            for inp in mesh_el.findall(f".//{_NS}input"):
                if inp.attrib.get("semantic") == "TEXCOORD":
                    uv_source_id = inp.attrib.get("source", "").lstrip("#")
                    break
            if uv_source_id:
                uv_src = mesh_el.find(f".//*[@id='{uv_source_id}']")
                if uv_src is not None:
                    uv_fa = uv_src.find(f"{_NS}float_array")
                    if uv_fa is not None and uv_fa.text:
                        uv_vals = np.fromstring(uv_fa.text, dtype=np.float32, sep=" ")
                        if len(uv_vals) >= 2:
                            uvs = uv_vals.reshape(-1, 2)
                            th, tw, _ = tex_arr.shape
                            if len(uvs) == len(mesh_coords):
                                u = np.clip(uvs[:, 0], 0.0, 1.0)
                                v = np.clip(1.0 - uvs[:, 1], 0.0, 1.0)
                                px_x = (u * (tw - 1)).astype(np.int32)
                                px_y = (v * (th - 1)).astype(np.int32)
                                mesh_colors = tex_arr[px_y, px_x]
                            else:
                                # Mapear mediante los índices <p> de <triangles>
                                tri_el = mesh_el.find(f"{_NS}triangles")
                                if tri_el is not None:
                                    p_el = tri_el.find(f"{_NS}p")
                                    inputs = tri_el.findall(f"{_NS}input")
                                    n_inputs = max(1, len(inputs))
                                    v_off, t_off = 0, 1
                                    for inp in inputs:
                                        sem = inp.attrib.get("semantic")
                                        off = int(inp.attrib.get("offset", 0))
                                        if sem == "VERTEX":
                                            v_off = off
                                        elif sem == "TEXCOORD":
                                            t_off = off
                                    if p_el is not None and p_el.text:
                                        p_idx = np.fromstring(p_el.text, dtype=np.int32, sep=" ")
                                        p_idx = p_idx.reshape(-1, n_inputs)
                                        v_indices = p_idx[:, v_off]
                                        t_indices = p_idx[:, t_off]
                                        valid = (
                                            (v_indices >= 0)
                                            & (v_indices < len(mesh_coords))
                                            & (t_indices >= 0)
                                            & (t_indices < len(uvs))
                                        )
                                        v_indices = v_indices[valid]
                                        t_indices = t_indices[valid]
                                        u = np.clip(uvs[t_indices, 0], 0.0, 1.0)
                                        v = np.clip(1.0 - uvs[t_indices, 1], 0.0, 1.0)
                                        px_x = (u * (tw - 1)).astype(np.int32)
                                        px_y = (v * (th - 1)).astype(np.int32)
                                        mesh_colors = np.zeros((len(mesh_coords), 3), dtype=np.uint8)
                                        mesh_colors[v_indices] = tex_arr[px_y, px_x]

        coords_list.append(mesh_coords)
        if mesh_colors is not None:
            colors_list.append(mesh_colors)

    if not coords_list:
        for fa in root.findall(f".//{_NS}float_array"):
            fid = fa.attrib.get("id", "").lower()
            if "pos" in fid and fa.text:
                vals = np.fromstring(fa.text, dtype=np.float32, sep=" ")
                if len(vals) >= 3:
                    coords_list.append(vals.reshape(-1, 3))

    if not coords_list:
        raise ValueError("No se encontraron geometrías 3D ni vértices en el archivo COLLADA (.dae).")

    coords = np.vstack(coords_list)
    colors = (
        np.vstack(colors_list)
        if (colors_list and len(colors_list) == len(coords_list))
        else None
    )

    # Eliminar vértices duplicados si los hubiera (muy común en mallas DAE desenrolladas por UV)
    if len(coords) > 100_000:
        _, unique_idx = np.unique(coords, axis=0, return_index=True)
        if len(unique_idx) < len(coords):
            unique_idx.sort()
            coords = coords[unique_idx]
            if colors is not None:
                colors = colors[unique_idx]

    if scale != 1.0:
        coords *= scale

    # Conversión de coordenadas hacia Z-up de IngeTrazo
    if up_axis == "Y_UP":
        coords = np.column_stack((coords[:, 0], -coords[:, 2], coords[:, 1]))
    elif up_axis == "X_UP":
        coords = np.column_stack((-coords[:, 1], coords[:, 0], coords[:, 2]))

    return coords, _sanitize_colors(colors)


def read_ply(path: str) -> tuple[np.ndarray, np.ndarray | None]:
    """Lee archivos PLY en formato ASCII o Binary Little Endian de forma vectorizada."""
    with open(path, "rb") as f:
        line = f.readline().decode("latin-1", errors="ignore").strip()
        if line != "ply":
            raise ValueError(f"El archivo {path} no es un formato PLY válido.")

        format_line = f.readline().decode("latin-1", errors="ignore").strip()
        is_binary = "binary_little_endian" in format_line
        is_ascii = "ascii" in format_line

        vertex_count = 0
        properties = []
        in_vertex_element = False

        while True:
            raw_hdr = f.readline()
            if not raw_hdr:
                break
            header_line = raw_hdr.decode("latin-1", errors="ignore").strip()
            if header_line.startswith("element"):
                parts = header_line.split()
                if len(parts) >= 3 and parts[1] == "vertex":
                    vertex_count = int(parts[2])
                    in_vertex_element = True
                else:
                    in_vertex_element = False
            elif header_line.startswith("property") and in_vertex_element:
                parts = header_line.split()
                if len(parts) >= 3 and parts[1] != "list":
                    prop_type, prop_name = parts[1], parts[2]
                    properties.append((prop_name, prop_type))
            elif header_line == "end_header":
                break

        prop_names = [p[0] for p in properties]
        n_props = len(prop_names)
        has_colors = all(c in prop_names for c in ("red", "green", "blue"))
        if not has_colors:
            has_colors = all(c in prop_names for c in ("r", "g", "b"))
            r_names = ("r", "g", "b")
        else:
            r_names = ("red", "green", "blue")

        if is_ascii:
            x_idx = prop_names.index("x") if "x" in prop_names else 0
            y_idx = prop_names.index("y") if "y" in prop_names else 1
            z_idx = prop_names.index("z") if "z" in prop_names else 2
            r_idx = prop_names.index(r_names[0]) if has_colors else -1
            g_idx = prop_names.index(r_names[1]) if has_colors else -1
            b_idx = prop_names.index(r_names[2]) if has_colors else -1

            # Lectura vectorizada ultrarrápida con np.fromstring
            body_text = f.read().decode("latin-1", errors="ignore")
            if n_props > 0 and vertex_count > 0:
                vals = np.fromstring(body_text, dtype=np.float32, sep=" ", count=vertex_count * n_props)
                actual_rows = len(vals) // n_props
                if actual_rows > 0:
                    mat = vals[: actual_rows * n_props].reshape(actual_rows, n_props)
                    coords = np.column_stack((mat[:, x_idx], mat[:, y_idx], mat[:, z_idx])).astype(np.float32)
                    if has_colors:
                        rc = mat[:, r_idx]
                        gc = mat[:, g_idx]
                        bc = mat[:, b_idx]
                        if rc.max() <= 1.0 and gc.max() <= 1.0 and bc.max() <= 1.0 and rc.max() > 0:
                            colors = np.column_stack((rc * 255, gc * 255, bc * 255)).astype(np.uint8)
                        else:
                            colors = np.column_stack((rc, gc, bc)).astype(np.uint8)
                    else:
                        colors = None
                    return coords, _sanitize_colors(colors)

            raise ValueError(f"No se pudieron extraer vértices del archivo PLY ASCII: {path}")

        elif is_binary:
            dtype_map = {
                "float": "<f4", "float32": "<f4",
                "double": "<f8", "float64": "<f8",
                "uchar": "u1", "uint8": "u1",
                "char": "i1", "int8": "i1",
                "int": "<i4", "int32": "<i4",
                "uint": "<u4", "uint32": "<u4",
                "short": "<i2", "int16": "<i2",
                "ushort": "<u2", "uint16": "<u2",
            }
            np_dtypes = [(name, dtype_map.get(ptype, "<f4")) for name, ptype in properties]
            rec_dtype = np.dtype(np_dtypes)
            vertex_data = np.fromfile(f, dtype=rec_dtype, count=vertex_count)

            coords = np.column_stack([
                vertex_data["x"].astype(np.float32),
                vertex_data["y"].astype(np.float32),
                vertex_data["z"].astype(np.float32),
            ])

            if has_colors:
                colors = np.column_stack([
                    vertex_data[r_names[0]].astype(np.uint8),
                    vertex_data[r_names[1]].astype(np.uint8),
                    vertex_data[r_names[2]].astype(np.uint8),
                ])
            else:
                colors = None
            return coords, _sanitize_colors(colors)
        else:
            raise ValueError("Formato PLY no soportado (debe ser ascii o binary_little_endian).")


def read_xyz(path: str) -> tuple[np.ndarray, np.ndarray | None]:
    """Lee archivos de texto .xyz, .pts, .txt o .csv (separados por espacios, comas o punto y coma)
    de forma vectorizada con NumPy.
    """
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        raw_text = f.read()

    if not raw_text:
        raise ValueError(f"El archivo {path} está vacío.")

    lines = raw_text.splitlines()
    n_cols = 0
    start_line_idx = 0
    for idx, line in enumerate(lines[:50]):
        s = line.strip()
        if not s or s.startswith(("#", "//", "X", "x")):
            continue
        clean_s = s.replace(",", " ").replace(";", " ")
        parts = clean_s.split()
        try:
            if len(parts) >= 3:
                float(parts[0])
                float(parts[1])
                float(parts[2])
                n_cols = len(parts)
                start_line_idx = idx
                break
            elif len(parts) == 1:
                int(parts[0])
                continue
        except ValueError:
            continue

    if n_cols >= 3:
        body = "\n".join(lines[start_line_idx:]) if start_line_idx > 0 else raw_text
        if "," in body[:1000] or ";" in body[:1000]:
            body = body.replace(",", " ").replace(";", " ")
        vals = np.fromstring(body, dtype=np.float32, sep=" ")
        n_rows = len(vals) // n_cols
        if n_rows > 0:
            mat = vals[: n_rows * n_cols].reshape(n_rows, n_cols)
            coords = mat[:, :3].astype(np.float32)
            colors = None
            if n_cols >= 6:
                rgb_start = 4 if n_cols >= 7 else 3
                r = mat[:, rgb_start]
                g = mat[:, rgb_start + 1]
                b = mat[:, rgb_start + 2]
                if r.max() <= 1.0 and g.max() <= 1.0 and b.max() <= 1.0 and r.max() > 0:
                    colors = np.column_stack((r * 255, g * 255, b * 255)).astype(np.uint8)
                else:
                    colors = np.column_stack((r, g, b)).astype(np.uint8)
            return coords, _sanitize_colors(colors)

    raise ValueError(f"No se pudieron leer puntos válidos del archivo {path}.")


def read_las(path: str) -> tuple[np.ndarray, np.ndarray | None]:
    """Lee archivos LiDAR .las de forma nativa con NumPy sin requerir librerías externas,
    con fallback a laspy para archivos comprimidos .laz.
    """
    ext = os.path.splitext(path)[1].lower()
    if ext == ".las":
        try:
            with open(path, "rb") as f:
                hdr = f.read(375)
                if len(hdr) >= 227 and hdr[:4] == b"LASF":
                    ver_major, ver_minor = hdr[24], hdr[25]
                    offset_to_pts = struct.unpack("<I", hdr[96:100])[0]
                    pt_format = hdr[104]
                    is_compressed = (pt_format & 0x80) != 0
                    pt_format &= 0x3F
                    pt_rec_len = struct.unpack("<H", hdr[105:107])[0]
                    num_pts = struct.unpack("<I", hdr[107:111])[0]
                    if ver_major == 1 and ver_minor >= 4 and num_pts == 0 and len(hdr) >= 255:
                        num_pts = struct.unpack("<Q", hdr[247:255])[0]
                    sx, sy, sz = struct.unpack("<ddd", hdr[131:155])
                    ox, oy, oz = struct.unpack("<ddd", hdr[155:179])

                    if not is_compressed and num_pts > 0 and pt_rec_len >= 20:
                        f.seek(offset_to_pts)
                        raw_bytes = f.read(num_pts * pt_rec_len)
                        actual_pts = len(raw_bytes) // pt_rec_len
                        if actual_pts > 0:
                            arr = np.frombuffer(
                                raw_bytes, dtype=np.uint8, count=actual_pts * pt_rec_len
                            ).reshape(actual_pts, pt_rec_len)
                            xyz_int = np.frombuffer(
                                arr[:, :12].tobytes(), dtype="<i4"
                            ).reshape(actual_pts, 3)
                            coords = np.empty((actual_pts, 3), dtype=np.float32)
                            coords[:, 0] = xyz_int[:, 0] * sx + ox
                            coords[:, 1] = xyz_int[:, 1] * sy + oy
                            coords[:, 2] = xyz_int[:, 2] * sz + oz

                            rgb_offset = {
                                2: 20,
                                3: 28,
                                5: 28,
                                7: 30,
                                8: 30,
                                10: 30,
                            }.get(pt_format)

                            colors = None
                            if rgb_offset is not None and pt_rec_len >= rgb_offset + 6:
                                rgb_u16 = np.frombuffer(
                                    arr[:, rgb_offset : rgb_offset + 6].tobytes(), dtype="<u2"
                                ).reshape(actual_pts, 3)
                                if rgb_u16.max() > 255:
                                    colors = (rgb_u16 >> 8).astype(np.uint8)
                                else:
                                    colors = rgb_u16.astype(np.uint8)

                            return coords, _sanitize_colors(colors)
        except Exception as e:
            _debug_log(f"Native LAS reader fallback: {e}")

    try:
        import laspy
        las = laspy.read(path)
        coords = np.column_stack((las.x, las.y, las.z)).astype(np.float32)

        colors = None
        if hasattr(las, "red") and hasattr(las, "green") and hasattr(las, "blue"):
            r = (las.red >> 8).astype(np.uint8) if las.red.max() > 255 else las.red.astype(np.uint8)
            g = (las.green >> 8).astype(np.uint8) if las.green.max() > 255 else las.green.astype(np.uint8)
            b = (las.blue >> 8).astype(np.uint8) if las.blue.max() > 255 else las.blue.astype(np.uint8)
            colors = np.column_stack((r, g, b))

        return coords, _sanitize_colors(colors)
    except ImportError:
        raise ValueError(
            "No se pudo abrir el archivo comprimido (.laz). Usa formato .las sin comprimir, .dae, .ply, .xyz o .e57."
        )


def _unpack_e57_bitpack(stream_bytes: bytes, count: int, bit_width: int) -> np.ndarray:
    """Desempaqueta un flujo de enteros sin signo empaquetados a nivel de bit (ASTM E57 BitPackCodec)."""
    if count <= 0 or bit_width <= 0:
        return np.zeros(max(0, count), dtype=np.int64)
    if bit_width == 8:
        return np.frombuffer(stream_bytes, dtype=np.uint8, count=count).astype(np.int64)
    if bit_width == 16:
        return np.frombuffer(stream_bytes, dtype="<u2", count=count).astype(np.int64)
    if bit_width == 32:
        return np.frombuffer(stream_bytes, dtype="<u4", count=count).astype(np.int64)

    arr = np.frombuffer(stream_bytes, dtype=np.uint8)
    padded = np.pad(arr, (0, 8), mode="constant")
    bit_pos = np.arange(count, dtype=np.int64) * bit_width
    byte_idx = bit_pos >> 3
    bit_rem = (bit_pos & 7).astype(np.uint64)

    u64 = (
        padded[byte_idx].astype(np.uint64)
        | (padded[byte_idx + 1].astype(np.uint64) << np.uint64(8))
        | (padded[byte_idx + 2].astype(np.uint64) << np.uint64(16))
        | (padded[byte_idx + 3].astype(np.uint64) << np.uint64(24))
        | (padded[byte_idx + 4].astype(np.uint64) << np.uint64(32))
        | (padded[byte_idx + 5].astype(np.uint64) << np.uint64(40))
    )
    mask = np.uint64((1 << bit_width) - 1)
    return ((u64 >> bit_rem) & mask).astype(np.int64)


def read_e57(path: str) -> tuple[np.ndarray, np.ndarray | None]:
    """Lee archivos estándar ASTM E57 (.e57) en Python puro + NumPy."""
    with open(path, "rb") as f:
        raw = f.read()

    if len(raw) < 48 or raw[:8] != b"ASTM-E57":
        raise ValueError(f"El archivo {path} no es un archivo ASTM E57 válido.")

    major, minor, file_len, xml_phys_off, xml_len, page_size = struct.unpack("<IIQQQQ", raw[8:48])
    if page_size <= 4:
        page_size = 1024
    data_per_page = page_size - 4

    full_pages = len(raw) // page_size
    if full_pages > 0:
        paged = np.frombuffer(raw[: full_pages * page_size], dtype=np.uint8).reshape(full_pages, page_size)
        logical_main = paged[:, :data_per_page].tobytes()
    else:
        logical_main = b""
    rem = raw[full_pages * page_size :]
    if rem:
        logical_main += rem[: min(len(rem), data_per_page)]

    def phys_to_log(p_off: int) -> int:
        return (p_off // page_size) * data_per_page + min(p_off % page_size, data_per_page)

    xml_log_off = phys_to_log(xml_phys_off)
    xml_bytes = logical_main[xml_log_off : xml_log_off + xml_len]
    xml_str = xml_bytes.decode("utf-8", errors="ignore").strip()
    end_idx = xml_str.find("</e57Root>")
    if end_idx != -1:
        xml_str = xml_str[: end_idx + len("</e57Root>")]

    root = ET.fromstring(xml_str)
    all_coords = []
    all_colors = []

    for pts_el in root.findall(".//{*}points"):
        file_offset = int(pts_el.attrib.get("fileOffset", "48"))
        record_count = int(pts_el.attrib.get("recordCount", "0"))
        if record_count <= 0:
            continue

        proto_el = pts_el.find("{*}prototype")
        if proto_el is None:
            continue

        fields = []
        for child in proto_el:
            tag = child.tag.split("}")[-1]
            ftype = child.attrib.get("type", "Float")
            fmin = float(child.attrib.get("minimum", "0"))
            fmax = float(child.attrib.get("maximum", "0"))
            fscale = float(child.attrib.get("scale", "1.0"))
            foffset = float(child.attrib.get("offset", "0.0"))
            fprec = child.attrib.get("precision", "single")

            if ftype in ("Integer", "ScaledInteger"):
                imin = int(round(fmin))
                imax = int(round(fmax))
                span = max(0, imax - imin)
                bits = span.bit_length() if span > 0 else 0
                fields.append({
                    "name": tag,
                    "type": ftype,
                    "imin": imin,
                    "imax": imax,
                    "bits": bits,
                    "scale": fscale,
                    "offset": foffset,
                })
            elif ftype == "Float":
                bits = 64 if fprec == "double" else 32
                fields.append({
                    "name": tag,
                    "type": "Float",
                    "bits": bits,
                    "fmin": fmin,
                    "fmax": fmax,
                })

        n_streams = len(fields)
        stream_buffers = [bytearray() for _ in range(n_streams)]

        sec_log_off = phys_to_log(file_offset)
        if sec_log_off + 32 > len(logical_main):
            continue
        sec_id, _, sec_len, data_phys_off, _ = struct.unpack(
            "<B7sQQQ", logical_main[sec_log_off : sec_log_off + 32]
        )
        pkt_log_off = phys_to_log(data_phys_off)
        sec_end = min(len(logical_main), sec_log_off + sec_len)

        while pkt_log_off + 6 <= sec_end:
            pkt_type, pkt_flags, pkt_len_m1, bs_count = struct.unpack(
                "<BBHH", logical_main[pkt_log_off : pkt_log_off + 6]
            )
            pkt_len = pkt_len_m1 + 1
            if pkt_type == 1:  # DataPacket
                if bs_count == n_streams and pkt_log_off + 6 + 2 * bs_count <= sec_end:
                    lens = struct.unpack(
                        f"<{bs_count}H",
                        logical_main[pkt_log_off + 6 : pkt_log_off + 6 + 2 * bs_count],
                    )
                    cur = pkt_log_off + 6 + 2 * bs_count
                    for s_idx, s_len in enumerate(lens):
                        if s_len > 0:
                            stream_buffers[s_idx].extend(logical_main[cur : cur + s_len])
                        cur += s_len
            elif pkt_type not in (0, 2):
                break
            pkt_log_off += pkt_len

        decoded = {}
        for s_idx, fld in enumerate(fields):
            name = fld["name"]
            ftype = fld["type"]
            bits = fld["bits"]
            buf = bytes(stream_buffers[s_idx])

            if ftype in ("Integer", "ScaledInteger"):
                if bits == 0:
                    vals = np.full(record_count, fld["imin"], dtype=np.int64)
                else:
                    max_avail = (len(buf) * 8) // bits
                    cnt = min(record_count, max_avail)
                    vals = _unpack_e57_bitpack(buf, cnt, bits) + fld["imin"]
                if ftype == "ScaledInteger":
                    decoded[name] = (vals * fld["scale"] + fld["offset"]).astype(np.float32)
                else:
                    decoded[name] = vals
            elif ftype == "Float":
                dt = "<f8" if bits == 64 else "<f4"
                b_per = 8 if bits == 64 else 4
                cnt = min(record_count, len(buf) // b_per)
                decoded[name] = np.frombuffer(buf, dtype=dt, count=cnt).astype(np.float32)

        if all(k in decoded for k in ("cartesianX", "cartesianY", "cartesianZ")):
            n_pts = min(len(decoded["cartesianX"]), len(decoded["cartesianY"]), len(decoded["cartesianZ"]))
            if n_pts > 0:
                c_xyz = np.column_stack((
                    decoded["cartesianX"][:n_pts],
                    decoded["cartesianY"][:n_pts],
                    decoded["cartesianZ"][:n_pts],
                )).astype(np.float32)
                all_coords.append(c_xyz)

                if all(k in decoded for k in ("colorRed", "colorGreen", "colorBlue")):
                    r = decoded["colorRed"][:n_pts]
                    g = decoded["colorGreen"][:n_pts]
                    b = decoded["colorBlue"][:n_pts]
                    fmax_r = next((f.get("fmax", 255.0) for f in fields if f["name"] == "colorRed"), 255.0)
                    if fmax_r <= 1.0 and r.max() <= 1.0:
                        c_rgb = np.column_stack((r * 255, g * 255, b * 255)).astype(np.uint8)
                    else:
                        c_rgb = np.column_stack((r, g, b)).astype(np.uint8)
                    all_colors.append(c_rgb)

    if not all_coords:
        raise ValueError(f"No se pudieron extraer puntos 3D del archivo E57 {path}.")

    coords = np.vstack(all_coords)
    colors = (
        np.vstack(all_colors)
        if (all_colors and len(all_colors) == len(all_coords))
        else None
    )
    return coords, _sanitize_colors(colors)


# ==============================================================================
# 2. CLASE PointCloud (GESTIÓN DE NUBE EN MEMORIA Y SINCRONIZACIÓN CON GRUPO)
# ==============================================================================

class PointCloud:
    """Gestiona una nube de puntos en memoria, vinculada a un Group nativo de IngeTrazo
    que representa su caja contenedora alámbrica.
    """

    def __init__(
        self,
        id: str,
        name: str,
        source_path: str,
        coords: np.ndarray,
        colors: np.ndarray | None = None,
    ) -> None:
        self.id = id
        self.name = name
        self.source_path = source_path
        self.coords = coords.astype(np.float32)
        self.colors = _sanitize_colors(colors)
        self.count = len(coords)

        # Copia local inmutable de referencia
        self.local_coords = self.coords.copy()

        # Límites iniciales
        if self.count > 0:
            self.min_bound = np.min(self.coords, axis=0)
            self.max_bound = np.max(self.coords, axis=0)
            self.center = (self.min_bound + self.max_bound) / 2.0
            self.box_size = self.max_bound - self.min_bound
        else:
            self.min_bound = np.zeros(3, dtype=np.float32)
            self.max_bound = np.zeros(3, dtype=np.float32)
            self.center = np.zeros(3, dtype=np.float32)
            self.box_size = np.zeros(3, dtype=np.float32)

        self.initial_bounds = (
            float(self.min_bound[0]), float(self.min_bound[1]), float(self.min_bound[2]),
            float(self.max_bound[0]), float(self.max_bound[1]), float(self.max_bound[2])
        )
        self.initial_c0 = np.array([self.initial_bounds[0], self.initial_bounds[1], self.initial_bounds[2]], dtype=np.float32)
        self.Lx = max(1e-4, float(self.initial_bounds[3] - self.initial_bounds[0]))
        self.Ly = max(1e-4, float(self.initial_bounds[4] - self.initial_bounds[1]))
        self.Lz = max(1e-4, float(self.initial_bounds[5] - self.initial_bounds[2]))

        # Referencia al Group nativo de IngeTrazo y sus 4 esquinas de base
        self.group: Group | None = None
        self.corners: tuple | None = None

        # Ajustes visuales (estilo Trimble Scan Essentials)
        self.visible = True
        self.locked = True  # Por defecto bloqueada como plantilla inerte
        self.point_size = 3
        self.color_mode = "rgb" if self.colors is not None else "height"
        self.single_color = (0.9, 0.9, 0.9)
        self.occlusion_enabled = True  # Ocultación de puntos tras caras sólidas

        # Mapa de Inspección / Análisis de Desviación (As-Built vs Modelo)
        self.inspection_enabled = False
        self.inspection_tolerance = 0.15  # 15 cm por defecto (ajustable de 1 cm a 100 cm)
        self.inspection_hide_far = False  # Si True, oculta puntos fuera de rango de inspección

        # Recorte de Sección (Clip horizontal)
        self.clip_z_enabled = False
        self.clip_z_min = float(self.min_bound[2])
        self.clip_z_max = float(self.max_bound[2])

        # Submuestreo rápido para interactividad fluida a 60+ FPS
        self._max_display_points = 500_000
        if self.count > self._max_display_points:
            step = int(math.ceil(self.count / self._max_display_points))
            self.coords_fast = self.local_coords[::step]
            self.colors_fast = self.colors[::step] if self.colors is not None else None
        else:
            self.coords_fast = self.local_coords
            self.colors_fast = self.colors

    def get_world_points(self, viewport, fast: bool = False) -> tuple[np.ndarray, np.ndarray | None]:
        """Calcula las coordenadas 3D de la nube en tiempo real siguiendo exactamente
        la posición y rotación actual del Group en la escena de IngeTrazo (incluyendo
        arrastres en vivo con la herramienta Mover y Rotar).
        """
        base_coords = self.coords_fast if fast else self.local_coords
        base_colors = self.colors_fast if fast else self.colors

        pts = self._transform_points(viewport, base_coords)

        if not self.clip_z_enabled:
            return pts, base_colors

        mask = (pts[:, 2] >= self.clip_z_min) & (pts[:, 2] <= self.clip_z_max)
        return pts[mask], (base_colors[mask] if base_colors is not None else None)

    def _transform_points(self, viewport, base_coords: np.ndarray) -> np.ndarray:
        """Aplica la transformación afín rígida derivada de las 4 esquinas del Group."""
        group = self.group
        if group is None or not group.mesh or len(group.mesh.vertices) < 5:
            return base_coords

        corners = getattr(self, "corners", None)
        if corners is None:
            verts = group.mesh.vertices
            if len(verts) >= 5:
                corners = (verts[0], verts[1], verts[3], verts[4])
                self.corners = corners
            else:
                return base_coords

        v0, v1, v3, v4 = corners
        P0 = QVector3D(v0.position)
        P1 = QVector3D(v1.position)
        P3 = QVector3D(v3.position)
        P4 = QVector3D(v4.position)

        if getattr(group, "xform", None) is not None:
            P0 = group.xform.map(P0)
            P1 = group.xform.map(P1)
            P3 = group.xform.map(P3)
            P4 = group.xform.map(P4)

        # Compensación de arrastre interactivo en vivo con herramientas Mover (M) y Rotar (Q)
        is_previewing = False
        sc = getattr(viewport, "scene", None)
        if sc and group in getattr(sc, "selection", []):
            is_previewing = True
        elif id(group) in getattr(viewport, "_preview_groups", set()):
            is_previewing = True

        if is_previewing:
            prev_mat = getattr(viewport, "_preview_matrix", None)
            prev_offset = getattr(viewport, "_preview_offset", None) or getattr(viewport, "_groups_preview_offset", None)
            if prev_mat is not None:
                P0 = prev_mat.map(P0)
                P1 = prev_mat.map(P1)
                P3 = prev_mat.map(P3)
                P4 = prev_mat.map(P4)
            elif prev_offset is not None:
                P0 += prev_offset
                P1 += prev_offset
                P3 += prev_offset
                P4 += prev_offset

        np_p0 = np.array([P0.x(), P0.y(), P0.z()], dtype=np.float32)
        np_p1 = np.array([P1.x(), P1.y(), P1.z()], dtype=np.float32)
        np_p3 = np.array([P3.x(), P3.y(), P3.z()], dtype=np.float32)
        np_p4 = np.array([P4.x(), P4.y(), P4.z()], dtype=np.float32)

        u = (np_p1 - np_p0) / self.Lx
        v = (np_p3 - np_p0) / self.Ly
        w = (np_p4 - np_p0) / self.Lz
        Basis = np.column_stack([u, v, w])

        transformed = (base_coords - self.initial_c0) @ Basis.T + np_p0
        return transformed.astype(np.float32)

    def is_screen_hit(self, viewport, screen_x: float, screen_y: float) -> bool:
        """Determina si las coordenadas de pantalla (screen_x, screen_y) tocan
        las aristas de la caja envolvente, sus esquinas o los puntos visibles de la nube.
        """
        if not self.visible or self.count == 0 or self.group is None:
            return False

        group = self.group
        if not group.mesh or len(group.mesh.vertices) < 5:
            return False

        # 1. Proximidad a los vértices (14 px) y aristas de la caja envolvente (12 px)
        w2p = getattr(viewport, "_world_to_pixel", None)
        if w2p is not None:
            verts = group.mesh.vertices
            proj_verts = []
            for v in verts:
                p = v.position
                if getattr(group, "xform", None) is not None:
                    p = group.xform.map(p)
                proj_verts.append(w2p(p))

            for pt in proj_verts:
                if pt is not None:
                    if math.hypot(pt[0] - screen_x, pt[1] - screen_y) <= 14.0:
                        return True

            edges_indices = [
                (0, 1), (1, 2), (2, 3), (3, 0),
                (4, 5), (5, 6), (6, 7), (7, 4),
                (0, 4), (1, 5), (2, 6), (3, 7),
                (0, 2)  # diagonal suelo
            ]
            click_pt = np.array([screen_x, screen_y], dtype=np.float32)
            for i, j in edges_indices:
                if i < len(proj_verts) and j < len(proj_verts):
                    p1, p2 = proj_verts[i], proj_verts[j]
                    if p1 is not None and p2 is not None:
                        a = np.array([p1[0], p1[1]], dtype=np.float32)
                        b = np.array([p2[0], p2[1]], dtype=np.float32)
                        ab = b - a
                        seg_len_sq = float(np.dot(ab, ab))
                        if seg_len_sq > 1e-4:
                            t = np.clip(np.dot(click_pt - a, ab) / seg_len_sq, 0.0, 1.0)
                            proj_p = a + t * ab
                            dist = float(np.linalg.norm(click_pt - proj_p))
                            if dist <= 12.0:
                                return True

        # 2. Proximidad a los puntos de la nube (14 px)
        pts, _ = self.get_world_points(viewport, fast=True)
        if len(pts) > 0:
            try:
                pt_px, pt_py, in_front = viewport.world_to_pixels(pts)
                thresh = 14.0
                mask = in_front & (pt_px >= screen_x - thresh) & (pt_px <= screen_x + thresh) \
                                & (pt_py >= screen_y - thresh) & (pt_py <= screen_y + thresh)
                if np.any(mask):
                    cand_pts = pts[mask]
                    cand_px = pt_px[mask]
                    cand_py = pt_py[mask]
                    dists = np.hypot(cand_px - screen_x, cand_py - screen_y)
                    best_i = int(np.argmin(dists))
                    if dists[best_i] <= thresh:
                        hit_p = cand_pts[best_i]
                        q_pt = QVector3D(float(hit_p[0]), float(hit_p[1]), float(hit_p[2]))
                        is_occ = getattr(viewport, "_is_occluded", None)
                        if is_occ and is_occ(q_pt):
                            return False
                        return True
            except Exception:
                pass

        return False


# ==============================================================================
# 3. RENDERIZADO OVERLAY, PROFUNDIDAD Y OCLUSIÓN DIRECTA (Trimble Scan Essentials)
# ==============================================================================

def render_clouds_overlay(viewport, painter: QPainter, clouds: list[PointCloud]) -> None:
    """Dibuja las nubes de puntos con oclusión precisa en profundidad contra caras sólidas.
    NOTA IMPORTANTE PARA MACOS: Nunca invocar funciones de OpenGL (_capture_depth / glBlitFramebuffer)
    mientras QPainter esté activo sobre QOpenGLWidget, ya que en macOS Cocoa invalida el
    framebuffer de Qt y deja el overlay en blanco. La captura de profundidad se solicita
    exclusivamente desde el hook de _annotations_need_depth() antes de que empiece QPainter.
    """
    if not clouds:
        return

    w = viewport.width()
    h = viewport.height()
    if w <= 0 or h <= 0:
        return

    depth_snap = getattr(viewport, "_depth_snap", None)

    cam = getattr(viewport, "camera", None)
    if cam is None:
        return

    try:
        M = viewport._np_mvp().astype(np.float32)
        inv_np = np.linalg.inv(M)
    except Exception:
        return

    eye_np = np.zeros(3, dtype=np.float32)
    fwd_np = np.array([0, 0, -1], dtype=np.float32)
    buf = None
    snap_w, snap_h = w, h

    if depth_snap is not None:
        try:
            buf, _mvp, _inv, snap_w, snap_h, eye, fwd = depth_snap
            eye_np = np.array([eye.x(), eye.y(), eye.z()], dtype=np.float32)
            fwd_np = np.array([fwd.x(), fwd.y(), fwd.z()], dtype=np.float32)
        except Exception:
            buf = None

    for cloud in clouds:
        if not cloud.visible or cloud.count == 0:
            continue

        pts, colors = cloud.get_world_points(viewport, fast=(cloud.count > 250_000))
        if len(pts) == 0:
            continue

        # Proyección matricial en lote (NumPy)
        clip = pts @ M[:, :3].T + M[:, 3]
        w_clip = clip[:, 3]
        in_front = w_clip > 1e-4
        if not np.any(in_front):
            continue

        safe_w = np.where(in_front, w_clip, 1.0)
        nx = clip[:, 0] / safe_w
        ny = clip[:, 1] / safe_w

        # Coordenadas en píxeles del widget
        px = (nx * 0.5 + 0.5) * w
        py = (1.0 - (ny * 0.5 + 0.5)) * h

        mask = in_front & (px >= -8) & (px <= w + 8) & (py >= -8) & (py <= h + 8)
        if not np.any(mask):
            continue

        sub_pts = pts[mask]
        sub_px = px[mask]
        sub_py = py[mask]
        sub_nx = nx[mask]
        sub_ny = ny[mask]

        # Coloreado (si se pide RGB pero el archivo no tiene colores válidos, usar Elevación Z)
        if cloud.color_mode == "rgb" and colors is not None:
            sub_c = colors[mask]
        elif cloud.color_mode in ("height", "rgb"):
            z_vals = sub_pts[:, 2]
            z_min = float(sub_pts[:, 2].min())
            z_max = float(sub_pts[:, 2].max())
            span = max(1e-4, z_max - z_min)
            t = np.clip((z_vals - z_min) / span, 0.0, 1.0)
            r = np.clip((t - 0.5) * 2.0 * 255, 0, 255).astype(np.uint8)
            g = (np.sin(t * np.pi) * 255).astype(np.uint8)
            b = np.clip((0.5 - t) * 2.0 * 255, 0, 255).astype(np.uint8)
            sub_c = np.column_stack((r, g, b))
        else:
            cr, cg, cb = [int(v * 255) for v in cloud.single_color]
            sub_c = np.full((len(sub_pts), 3), (cr, cg, cb), dtype=np.uint8)

        # Oclusión e Inspección de Desviación contra caras sólidas del modelo (Trimble Scan Essentials)
        occ_on = getattr(cloud, "occlusion_enabled", True)
        insp_on = getattr(cloud, "inspection_enabled", False)
        if (occ_on or insp_on) and buf is not None:
            ix = np.clip(((sub_nx + 1.0) * 0.5 * (snap_w - 1)).astype(np.int32), 0, snap_w - 1)
            iy = np.clip(((sub_ny + 1.0) * 0.5 * (snap_h - 1)).astype(np.int32), 0, snap_h - 1)

            depth_samples = buf[iy, ix]
            has_solid = depth_samples < 0.9999

            if np.any(has_solid):
                g_nx = sub_nx[has_solid]
                g_ny = sub_ny[has_solid]
                g_d = depth_samples[has_solid]
                g_pts = sub_pts[has_solid]

                g_ndc = np.column_stack([g_nx, g_ny, g_d * 2.0 - 1.0, np.ones(len(g_nx), dtype=np.float32)])
                g_surf_h = g_ndc @ inv_np.T
                safe_div = np.where(np.abs(g_surf_h[:, 3:4]) > 1e-9, g_surf_h[:, 3:4], 1.0)
                g_surf = g_surf_h[:, :3] / safe_div

                along_pt = (g_pts - eye_np) @ fwd_np
                along_surf = (g_surf - eye_np) @ fwd_np

                # 1. Mapa de Inspección (Semáforo de cercanía Verde -> Amarillo -> Rojo)
                if insp_on:
                    dist_to_surf = np.abs(along_pt - along_surf)
                    user_tol = max(0.005, float(getattr(cloud, "inspection_tolerance", 0.15)))
                    cutoff = max(user_tol * 2.5, user_tol + 0.35)
                    in_range = dist_to_surf <= cutoff

                    t = np.clip(dist_to_surf / user_tol, 0.0, 1.0)
                    # Gradiente suave: 0.0 = Verde (#00E632), 0.5 = Amarillo (#FFE600), 1.0 = Rojo (#FF1E1E)
                    r_hm = np.where(t < 0.5, t * 2.0 * 255.0, 255.0).astype(np.uint8)
                    g_hm = np.where(t < 0.5, 230.0, (1.0 - (t - 0.5) * 2.0) * 230.0).astype(np.uint8)
                    b_hm = np.where(t < 0.5, (1.0 - t * 2.0) * 50.0, 25.0).astype(np.uint8)
                    hm_colors = np.column_stack((r_hm, g_hm, b_hm))

                    solid_colors = sub_c[has_solid].copy()
                    if getattr(cloud, "inspection_hide_far", False):
                        solid_colors[in_range] = hm_colors[in_range]
                    else:
                        # Atenuar puntos lejanos sobre caras para destacar el mapa de calor
                        solid_colors[~in_range] = (solid_colors[~in_range].astype(np.uint16) * 45 // 100).astype(np.uint8)
                        solid_colors[in_range] = hm_colors[in_range]

                    # Atenuar también los puntos que proyectan fuera de cualquier cara sólida
                    if not getattr(cloud, "inspection_hide_far", False):
                        sub_c[~has_solid] = (sub_c[~has_solid].astype(np.uint16) * 45 // 100).astype(np.uint8)
                    sub_c[has_solid] = solid_colors

                # 2. Visibilidad (Oclusión y filtro de rango de inspección)
                vis_mask = np.ones(len(sub_pts), dtype=bool)
                if insp_on and getattr(cloud, "inspection_hide_far", False):
                    vis_mask[~has_solid] = False
                    solid_vis = dist_to_surf <= user_tol
                    if occ_on:
                        tol_occ = np.maximum(0.015, 0.003 * np.abs(along_pt))
                        occluded_solid = along_surf < (along_pt - tol_occ)
                        solid_vis = solid_vis & (~occluded_solid | (dist_to_surf <= user_tol))
                    vis_mask[has_solid] = solid_vis
                elif occ_on:
                    tol_occ = np.maximum(0.015, 0.003 * np.abs(along_pt))
                    occluded_solid = along_surf < (along_pt - tol_occ)
                    if insp_on:
                        # En modo inspección, mostrar los puntos dentro de la tolerancia aunque estén ligeramente detrás de la cara
                        occluded_solid = occluded_solid & (dist_to_surf > user_tol)
                    vis_mask[has_solid] = ~occluded_solid

                sub_px = sub_px[vis_mask]
                sub_py = sub_py[vis_mask]
                sub_c = sub_c[vis_mask]
            elif insp_on and getattr(cloud, "inspection_hide_far", False):
                # Si no hay ninguna cara sólida en pantalla y se pide ocultar fuera de rango
                continue

        if len(sub_px) > 0:
            pt_size = max(1, int(cloud.point_size))
            _draw_points_raster(painter, sub_px, sub_py, sub_c, w, h, pt_size)


def _draw_points_raster(
    painter: QPainter,
    px: np.ndarray,
    py: np.ndarray,
    colors: np.ndarray,
    w: int,
    h: int,
    size: int,
) -> None:
    """Rasteriza hasta cientos de miles de puntos directamente a un buffer QImage ultra-rápido."""
    buf = np.zeros((h, w, 4), dtype=np.uint8)

    ix = np.clip(np.round(px).astype(np.int32), 0, w - 1)
    iy = np.clip(np.round(py).astype(np.int32), 0, h - 1)

    buf[iy, ix, 0] = colors[:, 2]  # B
    buf[iy, ix, 1] = colors[:, 1]  # G
    buf[iy, ix, 2] = colors[:, 0]  # R
    buf[iy, ix, 3] = 255          # A

    if size >= 2:
        for dx in range(-size // 2, size // 2 + 1):
            for dy in range(-size // 2, size // 2 + 1):
                if dx == 0 and dy == 0:
                    continue
                nx = np.clip(ix + dx, 0, w - 1)
                ny = np.clip(iy + dy, 0, h - 1)
                buf[ny, nx, 0] = colors[:, 2]
                buf[ny, nx, 1] = colors[:, 1]
                buf[ny, nx, 2] = colors[:, 0]
                buf[ny, nx, 3] = 255

    qimg = QImage(buf.data, w, h, w * 4, QImage.Format_ARGB32_Premultiplied)
    painter.drawImage(0, 0, qimg.copy())


def snap_to_clouds(
    viewport,
    snap: SnapResult | None,
    px: float,
    py: float,
    clouds: list[PointCloud],
) -> SnapResult | None:
    """Permite que el cursor de cualquier herramienta (Línea, Rectángulo, Medir...)
    se enganche con imán (Snap) al punto de nube visible más cercano.
    """
    if not clouds:
        return None

    threshold = 16.0
    best_dist = threshold
    best_world = None

    for cloud in clouds:
        if not cloud.visible or cloud.count == 0:
            continue

        pts, _ = cloud.get_world_points(viewport, fast=True)
        if len(pts) == 0:
            continue

        try:
            pt_px, pt_py, in_front = viewport.world_to_pixels(pts)
        except Exception:
            continue

        valid = in_front & (pt_px >= px - threshold) & (pt_px <= px + threshold) \
                        & (pt_py >= py - threshold) & (pt_py <= py + threshold)
        if not np.any(valid):
            continue

        cand_px = pt_px[valid]
        cand_py = pt_py[valid]
        cand_pts = pts[valid]

        dx = cand_px - px
        dy = cand_py - py
        dists = np.sqrt(dx * dx + dy * dy)

        min_idx = np.argmin(dists)
        if dists[min_idx] < best_dist:
            best_dist = dists[min_idx]
            p = cand_pts[min_idx]
            best_world = QVector3D(float(p[0]), float(p[1]), float(p[2]))

    if best_world is None:
        return None

    try:
        return SnapResult(
            point=best_world,
            kind="endpoint",
            color=(0.95, 0.15, 0.85),
            label="Punto de nube"
        )
    except TypeError:
        return SnapResult(
            point=best_world,
            kind="endpoint",
            color=(0.95, 0.15, 0.85)
        )


# ==============================================================================
# 4. COMPONENTE UI PRINCIPAL (IngeScanWidget)
# ==============================================================================

class IngeScanWidget(QWidget):
    """Widget principal con todos los controles de escaneo, selección, bloqueo,
    alineación, oclusión y sección.
    """

    def __init__(self, viewport, app=None, is_dialog: bool = False, on_close=None, parent=None) -> None:
        super().__init__(parent)
        self.viewport = viewport
        self.app = app
        self.is_dialog = is_dialog
        self.on_close = on_close
        self.selected_cloud_id: str | None = None

        if not hasattr(self.viewport, "_ingescan_clouds"):
            self.viewport._ingescan_clouds = []

        self._cleanup_legacy_orphan_groups()
        self._setup_viewport_hooks()
        self._build_ui()
        self.refresh()

    @property
    def clouds(self) -> list[PointCloud]:
        return self.viewport._ingescan_clouds

    def _cleanup_legacy_orphan_groups(self) -> None:
        """Limpia cualquier grupo residual que no pertenezca a las nubes activas."""
        sc = getattr(self.viewport, "scene", None)
        if sc is None or not hasattr(sc, "groups"):
            return
        active_ids = {c.id for c in self.clouds}
        removed = 0
        for g in list(sc.groups):
            ext = getattr(g, "ext", {}) or {}
            gid = ext.get("ingescan", {}).get("id") or ext.get("ingescan_id")
            name = getattr(g, "name", "") or ""
            if (gid and gid not in active_ids) or (name.startswith("Escaneo:") and not gid):
                try:
                    sc.groups.remove(g)
                    removed += 1
                except Exception:
                    pass
        if removed > 0:
            _debug_log(f"Cleaned up {removed} legacy orphan scan groups from scene.")
            sc.version += 1
            self.viewport.notify_scene_changed()

    def _setup_viewport_hooks(self) -> None:
        """Conecta el renderizado de la nube, la oclusión y el Snap magnético al viewport."""
        vp = self.viewport

        # 1. Hook de oclusión e inspección en profundidad: asegura que paintGL capture el depth buffer
        if not getattr(vp, "_ingescan_depth_hooked", False):
            orig_annotations_need_depth = getattr(vp, "_annotations_need_depth", None)
            if callable(orig_annotations_need_depth):
                def hooked_annotations_need_depth():
                    if any(
                        (getattr(c, "occlusion_enabled", True) or getattr(c, "inspection_enabled", False))
                        and c.visible
                        for c in self.clouds
                    ):
                        return True
                    return orig_annotations_need_depth()
                vp._annotations_need_depth = hooked_annotations_need_depth
                vp._ingescan_depth_hooked = True

        # 2. Overlay oficial de dibujo
        if hasattr(vp, "_ext_overlays"):
            if self.draw_overlay not in vp._ext_overlays:
                vp._ext_overlays.append(self.draw_overlay)
        elif not getattr(vp, "_ingescan_overlay_hooked", False):
            orig_geo_points = getattr(vp, "_draw_geo_points", None)
            if callable(orig_geo_points):
                def wrapped_geo_points(painter):
                    orig_geo_points(painter)
                    self.draw_overlay(vp, painter)
                vp._draw_geo_points = wrapped_geo_points
                vp._ingescan_overlay_hooked = True

        # 3. Proveedor oficial de Snap magnético
        if hasattr(vp, "_ext_snap_providers"):
            if self.snap_provider not in vp._ext_snap_providers:
                vp._ext_snap_providers.append(self.snap_provider)

        # 4. Fallback hook de Snap para cuando snap inicial es None
        if not getattr(vp, "_ingescan_snap_hooked", False):
            orig_refresh_snap = getattr(vp, "_refresh_snap", None)
            if callable(orig_refresh_snap):
                def wrapped_refresh_snap():
                    orig_refresh_snap()
                    clouds = getattr(vp, "_ingescan_clouds", [])
                    if clouds and (vp.last_snap is None or getattr(vp.last_snap, "kind", "") in ("grid", "none", "plane")):
                        mpos = getattr(vp, "_last_mouse_pos", None)
                        if mpos is not None:
                            custom = snap_to_clouds(vp, vp.last_snap, mpos.x(), mpos.y(), clouds)
                            if custom is not None:
                                vp.last_snap = custom
                                at = getattr(vp, "active_tool", None)
                                if at is not None and hasattr(at, "on_hover"):
                                    try:
                                        ctx = ToolContext(
                                            viewport=vp,
                                            world=custom.point,
                                            screen=mpos,
                                            modifiers=QApplication.keyboardModifiers(),
                                            snap=custom,
                                        )
                                        at.on_hover(ctx)
                                    except Exception:
                                        pass
                vp._refresh_snap = wrapped_refresh_snap
                vp._ingescan_snap_hooked = True

        # 5. Hook de selección inteligente de nubes en viewport.pick_group
        if not getattr(vp, "_ingescan_pick_hooked", False):
            orig_pick_group = getattr(vp, "pick_group", None)
            if callable(orig_pick_group):
                def hooked_pick_group(screen_x: float, screen_y: float):
                    # Prioridad 1: Selección nativa de objetos/grupos estándar de IngeTrazo
                    res = orig_pick_group(screen_x, screen_y)
                    if res is not None:
                        return res

                    # Prioridad 2: Escaneos 3D y nubes de puntos de IngeScan
                    clouds = getattr(vp, "_ingescan_clouds", [])
                    for c in clouds:
                        if not c.visible or getattr(c, "group", None) is None:
                            continue
                        if c.is_screen_hit(vp, screen_x, screen_y):
                            if c.locked:
                                if hasattr(vp, "flash_status"):
                                    vp.flash_status(
                                        f"🔒 El escaneo '{c.name}' está fijado como plantilla. Desbloquéalo en el panel para moverlo con M o rotarlo con Q.",
                                        3500
                                    )
                                return None
                            return c.group
                    return None

                vp.pick_group = hooked_pick_group
                vp._ingescan_pick_hooked = True

    def draw_overlay(self, viewport, painter: QPainter) -> None:
        if self.clouds:
            sc = getattr(viewport, "scene", None)
            if sc and hasattr(sc, "groups"):
                # Sincronizar selección bidireccional desde scene.selection hacia la tabla
                if hasattr(sc, "selection"):
                    sel = sc.selection
                    for c in self.clouds:
                        if getattr(c, "group", None) in sel:
                            if self.selected_cloud_id != c.id:
                                self.selected_cloud_id = c.id
                                self.table.blockSignals(True)
                                for r, item in enumerate(self.clouds):
                                    if item.id == c.id:
                                        self.table.selectRow(r)
                                        break
                                self.table.blockSignals(False)
                                self._update_button_states()
                            break

            try:
                render_clouds_overlay(viewport, painter, self.clouds)
            except Exception as e:
                _debug_log(f"Error drawing overlay: {e}")

    def snap_provider(self, viewport, snap, px_x, px_y) -> SnapResult | None:
        if self.clouds:
            return snap_to_clouds(viewport, snap, px_x, px_y, self.clouds)
        return None

    def on_doc_changed(self) -> None:
        if getattr(self, "_is_importing", False):
            return
        self._cleanup_legacy_orphan_groups()
        self.refresh()
        self.viewport.update()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # Botón principal de Importar
        self.btn_import = QPushButton("📁 Importar escaneo 3D (DAE, PLY, XYZ, LAS, E57)...")
        self.btn_import.setStyleSheet("""
            QPushButton {
                background-color: #f37329;
                color: white;
                font-weight: bold;
                padding: 9px 12px;
                border-radius: 4px;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #e06318;
            }
        """)
        self.btn_import.clicked.connect(self._on_import_clicked)
        layout.addWidget(self.btn_import)

        # Tabla de escaneos cargados
        grp_list = QGroupBox("Escaneos activos en el modelo")
        list_layout = QVBoxLayout(grp_list)
        list_layout.setContentsMargins(4, 6, 4, 6)

        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["Nombre", "Puntos", "Ver", "Estado"])
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.itemSelectionChanged.connect(self._on_selection_changed)
        self.table.cellClicked.connect(self._on_cell_clicked)
        list_layout.addWidget(self.table)

        # Botón destacado de Bloqueo / Desbloqueo
        self.btn_toggle_lock = QPushButton("🔒 Bloqueado como plantilla")
        self.btn_toggle_lock.setToolTip(
            "Alterna entre bloquear como plantilla inerte (seguro para dibujar y medir encima sin moverlo) "
            "o desbloquear para seleccionarlo en el visor y moverlo con M o rotarlo con Q."
        )
        self.btn_toggle_lock.setStyleSheet("font-weight: bold; padding: 7px;")
        self.btn_toggle_lock.clicked.connect(self._on_toggle_lock_clicked)
        list_layout.addWidget(self.btn_toggle_lock)

        layout.addWidget(grp_list)

        # Caja de herramientas: Movimiento, Rotación y Alineación
        self.grp_transform = QGroupBox("Alineación y Posición rápida")
        trans_layout = QVBoxLayout(self.grp_transform)
        trans_layout.setContentsMargins(6, 8, 6, 8)
        trans_layout.setSpacing(6)

        # 1. Alineación rápida clara y diferenciada
        row_al1 = QHBoxLayout()
        self.btn_center_xy = QPushButton("⌖ Centrar en X=0, Y=0")
        self.btn_center_xy.setToolTip("Mueve el centro horizontal del escaneo al origen (0, 0), manteniendo la cota Z intacta.")
        self.btn_center_xy.clicked.connect(self._on_center_xy_clicked)
        row_al1.addWidget(self.btn_center_xy)

        self.btn_level = QPushButton("⤓ Apoyar suelo en Z = 0")
        self.btn_level.setToolTip("Ajusta la cota vertical para que el suelo más bajo del escaneo descanse exactamente en Z = 0.00 m.")
        self.btn_level.clicked.connect(self._on_level_ground_clicked)
        row_al1.addWidget(self.btn_level)
        trans_layout.addLayout(row_al1)

        self.btn_center_all = QPushButton("⌖ Centrar todo en (0, 0, Z=0)")
        self.btn_center_all.setToolTip("Centra horizontalmente en (0, 0) y coloca la base apoyada en Z = 0.00 m en un solo clic.")
        self.btn_center_all.clicked.connect(self._on_center_all_clicked)
        trans_layout.addWidget(self.btn_center_all)

        # 2. Giros ortogonales en planta (Z) y abatimiento de ejes verticales (X / Y)
        row_rot = QHBoxLayout()
        self.btn_rot_z90 = QPushButton("⟳ +90° Z")
        self.btn_rot_z90.setToolTip("Gira el escaneo +90° en planta respecto a su centro vertical.")
        self.btn_rot_z90.clicked.connect(lambda: self._apply_rotation_z(90.0))
        row_rot.addWidget(self.btn_rot_z90)

        self.btn_rot_zm90 = QPushButton("⟲ -90° Z")
        self.btn_rot_zm90.setToolTip("Gira el escaneo -90° en planta respecto a su centro vertical.")
        self.btn_rot_zm90.clicked.connect(lambda: self._apply_rotation_z(-90.0))
        row_rot.addWidget(self.btn_rot_zm90)

        self.btn_rot_z180 = QPushButton("180° Z")
        self.btn_rot_z180.setToolTip("Gira el escaneo 180° en planta.")
        self.btn_rot_z180.clicked.connect(lambda: self._apply_rotation_z(180.0))
        row_rot.addWidget(self.btn_rot_z180)
        trans_layout.addLayout(row_rot)

        row_rot_xy = QHBoxLayout()
        self.btn_rot_x90 = QPushButton("⤸ +90° X (Y↔Z)")
        self.btn_rot_x90.setToolTip("Abate el escaneo +90° sobre el eje X (intercambia ejes Y y Z si un archivo externo viene en Y-up o Z-up).")
        self.btn_rot_x90.clicked.connect(lambda: self._apply_rotation_axis("X", 90.0))
        row_rot_xy.addWidget(self.btn_rot_x90)

        self.btn_rot_xm90 = QPushButton("⤹ -90° X")
        self.btn_rot_xm90.setToolTip("Abate el escaneo -90° sobre el eje X.")
        self.btn_rot_xm90.clicked.connect(lambda: self._apply_rotation_axis("X", -90.0))
        row_rot_xy.addWidget(self.btn_rot_xm90)

        self.btn_rot_y90 = QPushButton("⤸ +90° Y")
        self.btn_rot_y90.setToolTip("Abate el escaneo +90° sobre el eje Y (intercambia ejes X y Z).")
        self.btn_rot_y90.clicked.connect(lambda: self._apply_rotation_axis("Y", 90.0))
        row_rot_xy.addWidget(self.btn_rot_y90)
        trans_layout.addLayout(row_rot_xy)

        # 3. Desplazamiento numérico por coordenadas
        row_shift = QHBoxLayout()
        row_shift.addWidget(QLabel("ΔX:"))
        self.spn_dx = QDoubleSpinBox()
        self.spn_dx.setRange(-500.0, 500.0)
        self.spn_dx.setSingleStep(0.2)
        self.spn_dx.setValue(0.0)
        self.spn_dx.setSuffix(" m")
        row_shift.addWidget(self.spn_dx)

        row_shift.addWidget(QLabel("ΔY:"))
        self.spn_dy = QDoubleSpinBox()
        self.spn_dy.setRange(-500.0, 500.0)
        self.spn_dy.setSingleStep(0.2)
        self.spn_dy.setValue(0.0)
        self.spn_dy.setSuffix(" m")
        row_shift.addWidget(self.spn_dy)

        row_shift.addWidget(QLabel("ΔZ:"))
        self.spn_dz = QDoubleSpinBox()
        self.spn_dz.setRange(-500.0, 500.0)
        self.spn_dz.setSingleStep(0.1)
        self.spn_dz.setValue(0.0)
        self.spn_dz.setSuffix(" m")
        row_shift.addWidget(self.spn_dz)

        self.btn_apply_shift = QPushButton("Mover")
        self.btn_apply_shift.clicked.connect(self._on_apply_shift_clicked)
        row_shift.addWidget(self.btn_apply_shift)
        trans_layout.addLayout(row_shift)

        # 4. Restablecer posición original
        self.btn_reset = QPushButton("↺ Restablecer a posición original")
        self.btn_reset.setToolTip("Devuelve la nube a las coordenadas exactas con las que fue importada.")
        self.btn_reset.clicked.connect(self._on_reset_clicked)
        trans_layout.addWidget(self.btn_reset)

        layout.addWidget(self.grp_transform)

        # Botones de Acción (Enfocar y Eliminar)
        row_actions = QHBoxLayout()
        self.btn_zoom = QPushButton("🔍 Enfocar en Visor")
        self.btn_zoom.clicked.connect(self._on_zoom_clicked)
        row_actions.addWidget(self.btn_zoom)

        self.btn_delete = QPushButton("🗑 Eliminar escaneo")
        self.btn_delete.setStyleSheet("""
            QPushButton {
                background-color: #d32f2f;
                color: white;
                font-weight: bold;
                padding: 6px 12px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #b71c1c;
            }
        """)
        self.btn_delete.clicked.connect(self._on_delete_clicked)
        row_actions.addWidget(self.btn_delete)
        layout.addLayout(row_actions)

        # Controles visuales (Trimble Scan Essentials)
        self.grp_controls = QGroupBox("Visualización y Oclusión")
        ctrl_layout = QVBoxLayout(self.grp_controls)
        ctrl_layout.setSpacing(6)

        # Oclusión inteligente por caras sólidas
        self.chk_occlusion = QCheckBox("Ocultar puntos tras caras sólidas (Oclusión)")
        self.chk_occlusion.setChecked(True)
        self.chk_occlusion.setToolTip(
            "Oculta automáticamente los puntos de la nube que queden dentro o detrás de prismas, muros o caras sólidas modeladas."
        )
        self.chk_occlusion.toggled.connect(self._on_occlusion_toggled)
        ctrl_layout.addWidget(self.chk_occlusion)

        # Tamaño de punto
        row_pt = QHBoxLayout()
        row_pt.addWidget(QLabel("Tamaño de punto:"))
        self.slider_pt = QSlider(Qt.Horizontal)
        self.slider_pt.setRange(1, 8)
        self.slider_pt.setValue(3)
        self.slider_pt.valueChanged.connect(self._on_point_size_changed)
        row_pt.addWidget(self.slider_pt)
        self.lbl_pt_val = QLabel("3 px")
        row_pt.addWidget(self.lbl_pt_val)
        ctrl_layout.addLayout(row_pt)

        # Modo de color
        row_col = QHBoxLayout()
        row_col.addWidget(QLabel("Modo de color:"))
        self.cmb_color = QComboBox()
        self.cmb_color.addItems([
            "Elevación / Altura (Z)",
            "Color original (RGB)",
            "Monocromo (Blanco)",
        ])
        self.cmb_color.currentIndexChanged.connect(self._on_color_mode_changed)
        row_col.addWidget(self.cmb_color)
        ctrl_layout.addLayout(row_col)

        # Separador
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setFrameShadow(QFrame.Sunken)
        ctrl_layout.addWidget(sep)

        # Corte por sección horizontal (planta de piso)
        self.chk_clip = QCheckBox("Activar corte de planta (Sección Z)")
        self.chk_clip.toggled.connect(self._on_clip_toggled)
        ctrl_layout.addWidget(self.chk_clip)

        row_z = QHBoxLayout()
        row_z.addWidget(QLabel("Z Mín:"))
        self.spn_z_min = QDoubleSpinBox()
        self.spn_z_min.setRange(-9999.0, 9999.0)
        self.spn_z_min.setSingleStep(0.2)
        self.spn_z_min.valueChanged.connect(self._on_z_clip_changed)
        row_z.addWidget(self.spn_z_min)

        row_z.addWidget(QLabel("Z Máx:"))
        self.spn_z_max = QDoubleSpinBox()
        self.spn_z_max.setRange(-9999.0, 9999.0)
        self.spn_z_max.setSingleStep(0.2)
        self.spn_z_max.valueChanged.connect(self._on_z_clip_changed)
        row_z.addWidget(self.spn_z_max)
        ctrl_layout.addLayout(row_z)

        # Separador hacia Mapa de Inspección
        sep_insp = QFrame()
        sep_insp.setFrameShape(QFrame.HLine)
        sep_insp.setFrameShadow(QFrame.Sunken)
        ctrl_layout.addWidget(sep_insp)

        # Mapa de Inspección / Análisis de Cercanía (Trimble Scan Essentials Inspection Map)
        self.chk_inspection = QCheckBox("🌡️ Activar Mapa de Inspección (Cercanía al modelo)")
        self.chk_inspection.setStyleSheet("font-weight: bold;")
        self.chk_inspection.setToolTip(
            "Compara los volúmenes y caras dibujadas con la nube de puntos, tiñendo los puntos de Verde (cerca / en tolerancia) "
            "a Rojo (lejos / fuera de tolerancia)."
        )
        self.chk_inspection.toggled.connect(self._on_inspection_toggled)
        ctrl_layout.addWidget(self.chk_inspection)

        # Contenedor desplegable de opciones de inspección
        self.pnl_inspection = QWidget()
        insp_layout = QVBoxLayout(self.pnl_inspection)
        insp_layout.setContentsMargins(10, 4, 4, 4)
        insp_layout.setSpacing(6)

        row_tol = QHBoxLayout()
        row_tol.addWidget(QLabel("Tolerancia:"))
        self.slider_tol = QSlider(Qt.Horizontal)
        self.slider_tol.setRange(1, 100)  # 1 cm (0.01 m) a 100 cm (1.00 m)
        self.slider_tol.setValue(15)      # 15 cm por defecto
        self.slider_tol.setToolTip("Distancia máxima de desviación entre tu modelo 3D y los puntos de la nube (Verde = 0 cm, Rojo = Tolerancia).")
        self.slider_tol.valueChanged.connect(self._on_inspection_tolerance_changed)
        row_tol.addWidget(self.slider_tol)

        self.lbl_tol_val = QLabel("15 cm (0.15 m)")
        self.lbl_tol_val.setMinimumWidth(88)
        row_tol.addWidget(self.lbl_tol_val)
        insp_layout.addLayout(row_tol)

        # Barra de leyenda visual del gradiente Verde -> Amarillo -> Rojo
        self.lbl_insp_legend = QLabel("0 cm (En cara)  ◄── 🟢 ── 🟡 ── 🔴 ──►  Límite")
        self.lbl_insp_legend.setAlignment(Qt.AlignCenter)
        self.lbl_insp_legend.setStyleSheet("""
            QLabel {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #00c832, stop:0.5 #ffe600, stop:1 #ff1e1e);
                color: #111111;
                font-weight: bold;
                font-size: 10px;
                padding: 4px 6px;
                border-radius: 3px;
                border: 1px solid #999999;
            }
        """)
        insp_layout.addWidget(self.lbl_insp_legend)

        self.chk_insp_hide_far = QCheckBox("Ocultar puntos fuera de la tolerancia")
        self.chk_insp_hide_far.setToolTip("Muestra únicamente los puntos cercanos a las caras modeladas dentro de la tolerancia elegida.")
        self.chk_insp_hide_far.toggled.connect(self._on_inspection_hide_far_toggled)
        insp_layout.addWidget(self.chk_insp_hide_far)

        self.pnl_inspection.setVisible(False)
        ctrl_layout.addWidget(self.pnl_inspection)

        layout.addWidget(self.grp_controls)

        # Botón Aceptar en diálogo flotante
        if self.is_dialog and self.on_close is not None:
            self.btn_accept = QPushButton("✔ Aceptar / Cerrar ventana")
            self.btn_accept.setStyleSheet("""
                QPushButton {
                    background-color: #2e7d32;
                    color: white;
                    font-weight: bold;
                    padding: 9px 14px;
                    border-radius: 4px;
                    font-size: 13px;
                }
                QPushButton:hover {
                    background-color: #1b5e20;
                }
            """)
            self.btn_accept.clicked.connect(self.on_close)
            layout.addWidget(self.btn_accept)

        # Nota de ayuda y autoría
        lbl_info = QLabel(
            "💡 <b>Manipulación en el Visor:</b> Cuando esté 🔓 <i>Desbloqueado</i>, puedes seleccionarlo en el visor "
            "con la herramienta <b>Seleccionar (Espacio)</b> y moverlo con <b>Mover (M)</b> o girarlo con <b>Rotar (Q)</b> "
            "o con los controles de este panel. Al terminar, pulsa 🔒 <i>Bloquear</i> para usarlo como plantilla fija."
        )
        lbl_info.setWordWrap(True)
        lbl_info.setStyleSheet("color: #777777; font-size: 11px;")
        layout.addWidget(lbl_info)

        lbl_author = QLabel("<b>IngeScan v1.0</b> · Desarrollado por <b>Jorge Rodríguez Pérez</b>")
        lbl_author.setAlignment(Qt.AlignCenter)
        lbl_author.setStyleSheet("color: #888888; font-size: 10px; padding-top: 4px;")
        layout.addWidget(lbl_author)

        layout.addStretch()

    def _get_selected_cloud(self) -> PointCloud | None:
        if not self.clouds:
            return None
        if self.selected_cloud_id:
            for c in self.clouds:
                if c.id == self.selected_cloud_id:
                    return c
        row = self.table.currentRow()
        if 0 <= row < len(self.clouds):
            return self.clouds[row]
        return self.clouds[0]

    def refresh(self) -> None:
        """Actualiza la tabla de escaneos y el estado de los controles de la UI."""
        self.table.blockSignals(True)
        self.table.setRowCount(len(self.clouds))

        sel_row = -1
        for row, cloud in enumerate(self.clouds):
            if cloud.id == self.selected_cloud_id:
                sel_row = row

            it_name = QTableWidgetItem(cloud.name)
            self.table.setItem(row, 0, it_name)

            it_count = QTableWidgetItem(f"{cloud.count:,}")
            self.table.setItem(row, 1, it_count)

            it_vis = QTableWidgetItem("👁" if cloud.visible else "✕")
            it_vis.setTextAlignment(Qt.AlignCenter)
            self.table.setItem(row, 2, it_vis)

            it_lock = QTableWidgetItem("🔒 Bloqueado" if cloud.locked else "🔓 Libre")
            it_lock.setTextAlignment(Qt.AlignCenter)
            if not cloud.locked:
                it_lock.setForeground(QColor(43, 87, 154))
            self.table.setItem(row, 3, it_lock)

        if sel_row >= 0:
            self.table.selectRow(sel_row)
        self.table.blockSignals(False)

        self._update_button_states()

    def _update_button_states(self) -> None:
        cloud = self._get_selected_cloud()
        if cloud:
            self.btn_toggle_lock.setEnabled(True)
            if cloud.locked:
                self.btn_toggle_lock.setText("🔒 Escaneo Bloqueado (Plantilla inerte)")
                self.btn_toggle_lock.setStyleSheet("background-color: #f0f0f0; color: #444; font-weight: bold; padding: 7px;")
            else:
                self.btn_toggle_lock.setText("🔓 Escaneo Desbloqueado (Listo para Mover/Rotar)")
                self.btn_toggle_lock.setStyleSheet("background-color: #e6f4ea; color: #137333; font-weight: bold; padding: 7px;")

            self.grp_transform.setEnabled(True)
            self.grp_transform.setToolTip("")

            self.slider_pt.setValue(int(cloud.point_size))
            self.lbl_pt_val.setText(f"{int(cloud.point_size)} px")

            modes = {"height": 0, "rgb": 1, "single": 2}
            self.cmb_color.blockSignals(True)
            self.cmb_color.setCurrentIndex(modes.get(cloud.color_mode, 0))
            self.cmb_color.blockSignals(False)

            self.chk_occlusion.blockSignals(True)
            self.chk_occlusion.setChecked(getattr(cloud, "occlusion_enabled", True))
            self.chk_occlusion.blockSignals(False)

            self.chk_clip.blockSignals(True)
            self.chk_clip.setChecked(cloud.clip_z_enabled)
            self.chk_clip.blockSignals(False)

            self.spn_z_min.blockSignals(True)
            self.spn_z_min.setValue(cloud.clip_z_min)
            self.spn_z_min.blockSignals(False)

            self.spn_z_max.blockSignals(True)
            self.spn_z_max.setValue(cloud.clip_z_max)
            self.spn_z_max.blockSignals(False)

            insp_on = getattr(cloud, "inspection_enabled", False)
            self.chk_inspection.blockSignals(True)
            self.chk_inspection.setChecked(insp_on)
            self.chk_inspection.blockSignals(False)
            self.pnl_inspection.setVisible(insp_on)

            tol_cm = max(1, min(100, int(round(getattr(cloud, "inspection_tolerance", 0.15) * 100))))
            self.slider_tol.blockSignals(True)
            self.slider_tol.setValue(tol_cm)
            self.slider_tol.blockSignals(False)
            self.lbl_tol_val.setText(f"{tol_cm} cm ({tol_cm / 100.0:.2f} m)")
            self.lbl_insp_legend.setText(f"0 cm (Verde)  ◄── 🟢 ── 🟡 ── 🔴 ──►  {tol_cm} cm (Rojo)")

            self.chk_insp_hide_far.blockSignals(True)
            self.chk_insp_hide_far.setChecked(getattr(cloud, "inspection_hide_far", False))
            self.chk_insp_hide_far.blockSignals(False)
        else:
            self.btn_toggle_lock.setEnabled(False)
            self.grp_transform.setEnabled(False)
            self.pnl_inspection.setVisible(False)

    def _on_selection_changed(self) -> None:
        row = self.table.currentRow()
        if 0 <= row < len(self.clouds):
            cloud = self.clouds[row]
            self.selected_cloud_id = cloud.id
            sc = getattr(self.viewport, "scene", None)
            if sc and hasattr(sc, "selection") and getattr(cloud, "group", None):
                if sc.selection != [cloud.group]:
                    sc.selection = [cloud.group]
                    self.viewport.update()
        self._update_button_states()

    def _on_cell_clicked(self, row: int, col: int) -> None:
        if 0 <= row < len(self.clouds):
            cloud = self.clouds[row]
            self.selected_cloud_id = cloud.id
            if col == 2:  # Ver / Ocultar
                cloud.visible = not cloud.visible
                self.refresh()
                self.viewport.update()
            elif col == 3:  # Bloquear / Desbloquear
                self._toggle_lock_cloud(cloud)

    def _on_toggle_lock_clicked(self) -> None:
        cloud = self._get_selected_cloud()
        if cloud:
            self._toggle_lock_cloud(cloud)

    def _toggle_lock_cloud(self, cloud: PointCloud) -> None:
        cloud.locked = not cloud.locked
        sc = getattr(self.viewport, "scene", None)
        if sc and getattr(cloud, "group", None):
            cloud.group.locked = cloud.locked
            sc.version += 1
            if not cloud.locked:
                sc.selection = [cloud.group]
            else:
                if cloud.group in getattr(sc, "selection", []):
                    sc.selection.remove(cloud.group)
            self.viewport.notify_scene_changed()

        self.refresh()
        self.viewport.update()

        status_msg = (
            f"🔒 Escaneo '{cloud.name}' fijado como plantilla inerte."
            if cloud.locked
            else f"🔓 Escaneo '{cloud.name}' desbloqueado y seleccionado en el visor. Puedes moverlo con M o rotarlo con Q."
        )
        if hasattr(self.viewport, "flash_status"):
            self.viewport.flash_status(status_msg, 4000)

    def _on_center_xy_clicked(self) -> None:
        """Centra el escaneo horizontalmente en X=0, Y=0 manteniendo la altura Z intacta."""
        cloud = self._get_selected_cloud()
        if not cloud or not getattr(cloud, "group", None):
            return
        verts = cloud.group.mesh.vertices
        xs = [v.position.x() for v in verts]
        ys = [v.position.y() for v in verts]
        cx = (min(xs) + max(xs)) / 2.0
        cy = (min(ys) + max(ys)) / 2.0
        self._translate_cloud_group(cloud, -cx, -cy, 0.0, f"Escaneo '{cloud.name}' centrado en X=0, Y=0 (Z sin alterar).")

    def _on_level_ground_clicked(self) -> None:
        """Nivela la base del escaneo para que su cota más baja quede en Z = 0.00 m."""
        cloud = self._get_selected_cloud()
        if not cloud or not getattr(cloud, "group", None):
            return
        verts = cloud.group.mesh.vertices
        zs = [v.position.z() for v in verts]
        min_z = min(zs)
        dz = -min_z
        self._translate_cloud_group(cloud, 0.0, 0.0, dz, f"Base del escaneo nivelada a Z = 0.00 m (desplazamiento vertical {dz:+.3f} m).")

    def _on_center_all_clicked(self) -> None:
        """Centra horizontalmente en (0, 0) y apoya la base en Z = 0.00 m en un solo clic."""
        cloud = self._get_selected_cloud()
        if not cloud or not getattr(cloud, "group", None):
            return
        verts = cloud.group.mesh.vertices
        xs = [v.position.x() for v in verts]
        ys = [v.position.y() for v in verts]
        zs = [v.position.z() for v in verts]
        cx = (min(xs) + max(xs)) / 2.0
        cy = (min(ys) + max(ys)) / 2.0
        min_z = min(zs)
        self._translate_cloud_group(cloud, -cx, -cy, -min_z, f"Escaneo '{cloud.name}' colocado en el origen (X=0, Y=0, Z=0.00 m).")

    def _apply_rotation_z(self, deg: float) -> None:
        """Gira la nube de puntos y su caja contenedora 90° o 180° en planta."""
        self._apply_rotation_axis("Z", deg)

    def _apply_rotation_axis(self, axis_name: str, deg: float) -> None:
        """Gira la nube de puntos y su caja contenedora alrededor de su centro 3D en el eje X, Y o Z."""
        cloud = self._get_selected_cloud()
        if not cloud or not getattr(cloud, "group", None):
            return
        group = cloud.group
        verts = group.mesh.vertices
        xs = [v.position.x() for v in verts]
        ys = [v.position.y() for v in verts]
        zs = [v.position.z() for v in verts]
        cx = (min(xs) + max(xs)) / 2.0
        cy = (min(ys) + max(ys)) / 2.0
        cz = (min(zs) + max(zs)) / 2.0
        center = QVector3D(cx, cy, cz)
        if axis_name == "X":
            axis = QVector3D(1.0, 0.0, 0.0)
        elif axis_name == "Y":
            axis = QVector3D(0.0, 1.0, 0.0)
        else:
            axis = QVector3D(0.0, 0.0, 1.0)
        m = rotation_matrix(center, axis, deg)
        for v in list(verts):
            group.mesh.move_vertex(v, m.map(v.position) - v.position)
        self.viewport.scene.version += 1
        self.viewport.notify_scene_changed()
        self.viewport.update()
        self.refresh()
        if hasattr(self.viewport, "flash_status"):
            self.viewport.flash_status(f"Escaneo rotado {deg:+.0f}° sobre el eje {axis_name}.", 3000)

    def _on_apply_shift_clicked(self) -> None:
        cloud = self._get_selected_cloud()
        if not cloud or not getattr(cloud, "group", None):
            return
        dx = self.spn_dx.value()
        dy = self.spn_dy.value()
        dz = self.spn_dz.value()
        if abs(dx) < 1e-4 and abs(dy) < 1e-4 and abs(dz) < 1e-4:
            return
        self._translate_cloud_group(cloud, dx, dy, dz, f"Desplazamiento aplicado ({dx:+.2f}, {dy:+.2f}, {dz:+.2f} m).")
        self.spn_dx.setValue(0.0)
        self.spn_dy.setValue(0.0)
        self.spn_dz.setValue(0.0)

    def _translate_cloud_group(self, cloud: PointCloud, dx: float, dy: float, dz: float, msg: str = "") -> None:
        group = cloud.group
        step = QVector3D(dx, dy, dz)
        for v in list(group.mesh.vertices):
            group.mesh.move_vertex(v, step)
        self.viewport.scene.version += 1
        self.viewport.notify_scene_changed()
        self.viewport.update()
        self.refresh()
        if msg and hasattr(self.viewport, "flash_status"):
            self.viewport.flash_status(msg, 3500)

    def _on_reset_clicked(self) -> None:
        cloud = self._get_selected_cloud()
        if not cloud or not getattr(cloud, "group", None):
            return
        group = cloud.group
        x0, y0, z0, x1, y1, z1 = cloud.initial_bounds
        orig_corners = [
            QVector3D(x0, y0, z0), QVector3D(x1, y0, z0), QVector3D(x1, y1, z0), QVector3D(x0, y1, z0),
            QVector3D(x0, y0, z1), QVector3D(x1, y0, z1), QVector3D(x1, y1, z1), QVector3D(x0, y1, z1),
        ]
        for i, v in enumerate(group.mesh.vertices[:8]):
            delta = orig_corners[i] - v.position
            group.mesh.move_vertex(v, delta)
        self.viewport.scene.version += 1
        self.viewport.notify_scene_changed()
        self.viewport.update()
        self.refresh()
        if hasattr(self.viewport, "flash_status"):
            self.viewport.flash_status(f"Escaneo '{cloud.name}' restablecido a su posición original.", 3500)

    def _on_occlusion_toggled(self, checked: bool) -> None:
        cloud = self._get_selected_cloud()
        if cloud:
            cloud.occlusion_enabled = checked
            self.viewport.update()

    def _on_point_size_changed(self, val: int) -> None:
        self.lbl_pt_val.setText(f"{val} px")
        cloud = self._get_selected_cloud()
        if cloud:
            cloud.point_size = val
            self.viewport.update()

    def _on_color_mode_changed(self, idx: int) -> None:
        modes = ["height", "rgb", "single"]
        cloud = self._get_selected_cloud()
        if cloud and idx < len(modes):
            cloud.color_mode = modes[idx]
            self.viewport.update()

    def _on_clip_toggled(self, checked: bool) -> None:
        cloud = self._get_selected_cloud()
        if cloud:
            cloud.clip_z_enabled = checked
            self.viewport.update()

    def _on_z_clip_changed(self) -> None:
        cloud = self._get_selected_cloud()
        if cloud:
            cloud.clip_z_min = self.spn_z_min.value()
            cloud.clip_z_max = self.spn_z_max.value()
            self.viewport.update()

    def _on_inspection_toggled(self, checked: bool) -> None:
        self.pnl_inspection.setVisible(checked)
        cloud = self._get_selected_cloud()
        if cloud:
            cloud.inspection_enabled = checked
            self.viewport.update()
            if hasattr(self.viewport, "flash_status"):
                if checked:
                    tol_cm = int(round(cloud.inspection_tolerance * 100))
                    self.viewport.flash_status(
                        f"🌡️ Mapa de Inspección activado (Tolerancia: {tol_cm} cm). Dibuja o coloca un volumen sólido junto a la nube para ver las desviaciones.",
                        4500,
                    )
                else:
                    self.viewport.flash_status("Mapa de Inspección desactivado.", 2500)

    def _on_inspection_tolerance_changed(self, val_cm: int) -> None:
        tol_m = max(0.01, val_cm / 100.0)
        self.lbl_tol_val.setText(f"{val_cm} cm ({tol_m:.2f} m)")
        self.lbl_insp_legend.setText(f"0 cm (Verde)  ◄── 🟢 ── 🟡 ── 🔴 ──►  {val_cm} cm (Rojo)")
        cloud = self._get_selected_cloud()
        if cloud:
            cloud.inspection_tolerance = tol_m
            self.viewport.update()

    def _on_inspection_hide_far_toggled(self, checked: bool) -> None:
        cloud = self._get_selected_cloud()
        if cloud:
            cloud.inspection_hide_far = checked
            self.viewport.update()

    def _on_zoom_clicked(self) -> None:
        cloud = self._get_selected_cloud()
        if cloud:
            self._zoom_to_cloud(cloud)

    def _zoom_to_cloud(self, cloud: PointCloud) -> None:
        if not hasattr(self.viewport, "camera") or not getattr(cloud, "group", None):
            return
        cam = self.viewport.camera
        verts = cloud.group.mesh.vertices
        xs = [v.position.x() for v in verts]
        ys = [v.position.y() for v in verts]
        zs = [v.position.z() for v in verts]
        min_v = QVector3D(min(xs), min(ys), min(zs))
        max_v = QVector3D(max(xs), max(ys), max(zs))
        if hasattr(cam, "fit_to"):
            try:
                cam.fit_to(min_v, max_v, margin=1.2)
            except Exception:
                pass
        elif hasattr(cam, "target"):
            cam.target = (min_v + max_v) * 0.5
        self.viewport.update()

    def _on_delete_clicked(self) -> None:
        cloud = self._get_selected_cloud()
        if not cloud:
            QMessageBox.information(self, "IngeScan", "No hay ningún escaneo seleccionado para eliminar.")
            return

        ans = QMessageBox.question(
            self,
            "Eliminar escaneo",
            f"¿Deseas eliminar definitivamente el escaneo '{cloud.name}' del modelo?",
            QMessageBox.Yes | QMessageBox.No,
        )
        if ans != QMessageBox.Yes:
            return

        self.delete_cloud(cloud)

    def delete_cloud(self, cloud: PointCloud) -> None:
        sc = getattr(self.viewport, "scene", None)
        if sc and getattr(cloud, "group", None):
            if cloud.group in sc.groups:
                sc.groups.remove(cloud.group)
            if cloud.group in getattr(sc, "selection", []):
                sc.selection.remove(cloud.group)
            sc.version += 1
            self.viewport.notify_scene_changed()

        if cloud in self.clouds:
            self.clouds.remove(cloud)
        if self.selected_cloud_id == cloud.id:
            self.selected_cloud_id = self.clouds[0].id if self.clouds else None

        self.refresh()
        self.viewport.update()
        if hasattr(self.viewport, "flash_status"):
            self.viewport.flash_status(f"Escaneo '{cloud.name}' eliminado completamente.", 3000)

    def _on_import_clicked(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Seleccionar escaneo 3D o nube de puntos",
            "",
            "Archivos de escaneo (*.dae *.ply *.xyz *.pts *.txt *.csv *.las *.e57);;COLLADA (*.dae);;Nubes PLY (*.ply);;Texto XYZ (*.xyz *.pts *.txt *.csv);;LiDAR LAS (*.las);;ASTM E57 (*.e57)",
        )
        if not path:
            return

        self.import_cloud(path)

    def import_cloud(self, path: str) -> None:
        """Importa el archivo de puntos, crea su Group alámbrico en la escena y lo enfoca en cámara."""
        _debug_log(f"Importing cloud from: {path}")
        self._is_importing = True
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            coords, colors = read_file(path)
            name = os.path.splitext(os.path.basename(path))[0]
            cid = f"cloud_{int(time.time() * 1000)}"
            cloud = PointCloud(id=cid, name=name, source_path=path, coords=coords, colors=colors)

            # Asegurar que los hooks del viewport siguen activos
            self._setup_viewport_hooks()

            # Registrar primero en self.clouds antes de notificar a la escena
            self.clouds.append(cloud)
            self.selected_cloud_id = cloud.id

            # Crear el Group alámbrico nativo en IngeTrazo
            self._create_cloud_group(cloud)

            self.refresh()
            self._zoom_to_cloud(cloud)
            self.viewport.update()

            if hasattr(self.viewport, "flash_status"):
                self.viewport.flash_status(
                    f"✓ Escaneo '{name}' importado ({cloud.count:,} puntos). Bloqueado como plantilla inerte.",
                    4500
                )
            _debug_log(f"Cloud '{name}' imported successfully. Total points: {cloud.count}")
        except Exception as e:
            import traceback
            err_tb = traceback.format_exc()
            _debug_log(f"Error importing cloud from {path}:\n{err_tb}")
            QMessageBox.critical(self, "Error al importar", f"No se pudo cargar el escaneo:\n{e}")
        finally:
            self._is_importing = False
            while QApplication.overrideCursor() is not None:
                QApplication.restoreOverrideCursor()

    def _create_cloud_group(self, cloud: PointCloud) -> None:
        """Crea un Group nativo en scene.groups con las aristas alámbricas de la caja envolvente.
        NO tiene caras (mesh.faces está vacío), evitando cualquier plano blanco indeseado.
        """
        sc = getattr(self.viewport, "scene", None)
        if sc is None:
            return

        x0, y0, z0, x1, y1, z1 = cloud.initial_bounds

        c0 = QVector3D(x0, y0, z0)
        c1 = QVector3D(x1, y0, z0)
        c2 = QVector3D(x1, y1, z0)
        c3 = QVector3D(x0, y1, z0)
        c4 = QVector3D(x0, y0, z1)
        c5 = QVector3D(x1, y0, z1)
        c6 = QVector3D(x1, y1, z1)
        c7 = QVector3D(x0, y1, z1)

        mesh = Mesh()
        # 12 aristas de la caja envolvente
        mesh.add_edge(c0, c1)  # v0, v1
        mesh.add_edge(c1, c2)  # v1, v2
        mesh.add_edge(c2, c3)  # v2, v3
        mesh.add_edge(c3, c0)
        mesh.add_edge(c4, c5)  # v4, v5
        mesh.add_edge(c5, c6)  # v5, v6
        mesh.add_edge(c6, c7)  # v6, v7
        mesh.add_edge(c7, c4)
        mesh.add_edge(c0, c4)
        mesh.add_edge(c1, c5)
        mesh.add_edge(c2, c6)
        mesh.add_edge(c3, c7)
        # Diagonal en el suelo para facilitar el clic de selección en planta
        mesh.add_edge(c0, c2)

        group = Group(mesh=mesh, name=f"Escaneo: {cloud.name}")
        group.ext = {"ingescan": {"id": cloud.id}}
        group.locked = cloud.locked

        # Guardar referencias fijas a los vértices de esquina en el objeto PointCloud
        # (NUNCA como atributo arbitrario en group porque Group usa __slots__)
        verts = mesh.vertices
        cloud.corners = (verts[0], verts[1], verts[3], verts[4])

        cloud.group = group

        sc.groups.append(group)
        sc.version += 1
        self.viewport.notify_scene_changed()


# ==============================================================================
# 5. DIÁLOGO FLOTANTE (FALLBACK PARA VERSIONES ANTIGUAS DE INGETRAZO)
# ==============================================================================

class IngeScanDialog(QDialog):
    """Diálogo flotante que contiene el panel de IngeScan cuando se ejecuta
    en versiones de IngeTrazo que no disponen de bandeja dock de extensiones.
    """

    def __init__(self, viewport, app=None, parent=None) -> None:
        super().__init__(parent or getattr(viewport, "window", lambda: None)())
        self.setWindowTitle("IngeScan — Nubes de Puntos y Escaneos 3D")
        self.resize(380, 680)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        self.widget = IngeScanWidget(viewport, app=app, is_dialog=True, on_close=self.accept, parent=self)
        layout.addWidget(self.widget)


class IngeScanTool(Tool):
    """Herramienta disparadora desde el menú Extensiones."""
    name = "IngeScan"
    shortcut = None
    description = "Abre el panel de nubes de puntos y escaneos 3D IngeScan"

    def on_activate(self, viewport) -> None:
        win = getattr(viewport, "window", lambda: None)()
        dock = getattr(win, "_ingescan_dock", None)
        app = getattr(win, "_extension_app", None)
        if dock and app and hasattr(app, "show_panel"):
            app.show_panel(dock)
        else:
            dialog = IngeScanDialog(viewport, app=app, parent=win)
            dialog.show()
            dialog.raise_()
            dialog.activateWindow()

    def on_deactivate(self, viewport) -> None:
        pass


# ==============================================================================
# 6. REGISTRO Y PUNTO DE ENTRADA (ExtensionApp setup)
# ==============================================================================

def setup(app) -> None:
    """Punto de entrada oficial ExtensionApp para IngeTrazo v0.5.3 a v0.5.8+.
    Agrega la pestaña de IngeScan en la bandeja lateral (junto a Propiedades/BIM),
    registra el overlay oficial y el imán magnético (Snap).
    """
    _debug_log("setup(app) called successfully.")
    try:
        vp = app.viewport
        widget = IngeScanWidget(vp, app=app, is_dialog=False)

        # 1. Panel lateral dock en la bandeja de pestañas
        dock = None
        if hasattr(app, "add_panel"):
            dock = app.add_panel("IngeScan", widget)
            app.window._ingescan_dock = dock
            app.window._extension_app = app
            app.show_panel(dock)

        # 2. Overlay oficial de renderizado de la nube sobre el visor
        if hasattr(app, "add_overlay"):
            if widget.draw_overlay not in getattr(vp, "_ext_overlays", []):
                app.add_overlay(widget.draw_overlay)

        # 3. Proveedor oficial de Snap magnético para dibujar sobre la nube
        if hasattr(app, "add_snap_provider"):
            if widget.snap_provider not in getattr(vp, "_ext_snap_providers", []):
                app.add_snap_provider(widget.snap_provider)

        # 4. Acción en menú Extensiones para traer el panel al frente
        if hasattr(app, "add_menu_action"):
            app.add_menu_action(
                "IngeScan: Nubes de puntos…",
                lambda: app.show_panel(dock) if dock else IngeScanTool().on_activate(vp),
                tip="Abre el panel de nubes de puntos y escaneos 3D IngeScan",
            )

        # 5. Sincronización de eventos de documento
        if hasattr(app, "on_document_changed"):
            app.on_document_changed(widget.on_doc_changed)

    except Exception as e:
        _debug_log(f"setup(app) error: {e}")

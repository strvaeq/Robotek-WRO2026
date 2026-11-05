"""Deteccion de bloques rojos y verdes por HSV. """
import cv2
import numpy as np

LIBRE, ROJO, VERDE = 0, 1, 2


def _mask(hsv, color, cfg):
    if color == ROJO:
        # el rojo cruza el limite del matiz, asi que se unen dos rangos
        m1 = cv2.inRange(hsv, (cfg.RED_H1_LOW, cfg.RED_S_MIN, cfg.RED_V_MIN),
                         (cfg.RED_H1_HIGH, 255, 255))
        m2 = cv2.inRange(hsv, (cfg.RED_H2_LOW, cfg.RED_S_MIN, cfg.RED_V_MIN),
                         (cfg.RED_H2_HIGH, 255, 255))
        mask = cv2.bitwise_or(m1, m2)
    else:
        mask = cv2.inRange(hsv, (cfg.GREEN_H_LOW, cfg.GREEN_S_MIN, cfg.GREEN_V_MIN),
                           (cfg.GREEN_H_HIGH, 255, 255))
    k = np.ones((cfg.COLOR_MORPH_KERNEL, cfg.COLOR_MORPH_KERNEL), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=cfg.COLOR_MORPH_ITERS)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=cfg.COLOR_MORPH_ITERS)


def _blocks(mask, color, width, height, cfg):
    """Contornos que pasan los filtros de tamano/forma, como bloques validos."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blocks = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < cfg.COLOR_MIN_AREA_PX:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if w == 0 or h == 0:
            continue
        if area / (w * h) < cfg.COLOR_MIN_FILL:          # descarta manchas dispersas
            continue
        if max(w, h) / min(w, h) > cfg.COLOR_MAX_WIDTH_HEIGHT:
            continue
        cx_norm = (x + w / 2.0) / width - 0.5            # -0.5 izq ... +0.5 der
        base_norm = (y + h) / height                     # 1.0 = borde inferior = cerca
        blocks.append((color, cx_norm, base_norm, float(area)))
    return blocks


def detect_block(frame, cfg):
    """Bloque mas cercano como (color, cx_norm, base_norm, area), o None.

    cx_norm < 0 = a la izquierda del centro. base_norm alto = mas cerca (su
    base cae mas abajo en la imagen).
    """
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        return None
    height, width = frame.shape[:2]
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
    blocks = (_blocks(_mask(hsv, ROJO, cfg), ROJO, width, height, cfg) +
              _blocks(_mask(hsv, VERDE, cfg), VERDE, width, height, cfg))
    if not blocks:
        return None
    return max(blocks, key=lambda b: b[2])

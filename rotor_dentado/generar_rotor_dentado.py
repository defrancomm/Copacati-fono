# -*- coding: utf-8 -*-
"""
Rotor cónico dentado (maqueta para practicar bobinado) - generador CadQuery.

Genera un rotor troncocónico con ranuras semicerradas (dientes con zapata)
a lo largo de la generatriz del cono, un cubo con prisionero M3 en el extremo
mayor y el alojamiento pasante del eje. Exporta:

    salida/Rotor_Conico_Dentado.step   -> abrir en Fusion 360 (sólido)
    salida/Rotor_Conico_Dentado.stl    -> laminador
    salida/Rotor_Conico_Dentado.3mf    -> laminador

Orientación (ya lista para imprimir): extremo MENOR en z = 0 (cama),
extremo MAYOR arriba y el cubo encima. Todas las caras quedan a menos de
10° de la vertical, así que no necesita soportes.

Uso:  python generar_rotor_dentado.py
"""

import math
import os

import cadquery as cq

# ---------------------------------------------------------------------------
# Parámetros (mm). Las cotas de la ranura se definen en el extremo MAYOR y se
# escalan linealmente hacia el menor (ranuras cónicas, como una chapa real).
# ---------------------------------------------------------------------------
ROTOR_HEIGHT = 60.0
LARGE_DIAMETER = 50.0
SMALL_DIAMETER = 30.0
SHAFT_DIAMETER = 8.0
SHAFT_CLEARANCE = 0.2
SHAFT_HOLE_DIAMETER = SHAFT_DIAMETER + SHAFT_CLEARANCE
SHAFT_ENTRY_CHAMFER = 0.5

NUM_SLOTS = 12              # número de ranuras = número de dientes
SLOT_DEPTH = 9.0            # del diámetro exterior al fondo de ranura
TOOTH_WIDTH = 4.5           # ancho del cuerpo del diente (lados paralelos)
SLOT_OPENING = 3.0          # boca de la ranura entre zapatas
SHOE_TIP_THICKNESS = 1.8    # espesor radial de la zapata en la punta
SHOE_ROOT_THICKNESS = 3.2   # espesor radial de la zapata en la unión con el diente

TOP_FILLET = 0.8            # redondeo del contorno en el extremo mayor (protege el esmalte)
BOTTOM_CHAMFER = 0.4        # chaflán del contorno en la cara de apoyo (pata de elefante)

HUB_DIAMETER = 22.0         # cubo para el prisionero y para sujetar la pieza al bobinar
HUB_LENGTH = 10.0
HUB_FILLET = 1.0
SET_SCREW_HOLE_DIAMETER = 2.5   # broca de roscar M3 (rosca directa o autorroscado)

MIN_WALL = 2.0
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'salida')
NAME = 'Rotor_Conico_Dentado'


def radius_at(z):
    """Radio exterior del cono a la altura z (z = 0 extremo menor)."""
    r_s, r_l = SMALL_DIAMETER / 2, LARGE_DIAMETER / 2
    return r_s + (r_l - r_s) * z / ROTOR_HEIGHT


def flank_point(r, sign):
    """Punto del flanco del diente adyacente a la ranura (centrada en +X) a radio r.
    sign = +1 diente superior, -1 diente inferior."""
    a = math.pi / NUM_SLOTS                     # eje del diente adyacente
    d = TOOTH_WIDTH / 2
    t = math.sqrt(r * r - d * d)
    x = t * math.cos(a) + d * math.sin(a)
    y = t * math.sin(a) - d * math.cos(a)
    return x, sign * y


def slot_polygon(z):
    """Vértices de una ranura (centrada en +X) a la altura z, escalados con el cono.
    El fondo es una cuerda: su flecha respecto al arco es ~0.1 mm."""
    k = radius_at(z) / (LARGE_DIAMETER / 2)
    r_l = LARGE_DIAMETER / 2
    r_b = r_l - SLOT_DEPTH
    r_s = r_l - SHOE_ROOT_THICKNESS
    r_t = r_l - SHOE_TIP_THICKNESS
    w = SLOT_OPENING / 2
    x_t = math.sqrt(r_t * r_t - w * w)
    r_ext = r_l + 3.0
    pts = [
        flank_point(r_b, -1), flank_point(r_b, +1),     # fondo de ranura
        flank_point(r_s, +1), (x_t, w),                 # flanco y zapata superior
        (r_ext, w), (r_ext, -w),                        # boca (sale del cono)
        (x_t, -w), flank_point(r_s, -1),                # zapata y flanco inferior
    ]
    return [cq.Vector(x * k, y * k, z) for x, y in pts]


def slot_cutter(z0, z1):
    """Sólido de corte de una ranura entre z0 y z1.

    Como las secciones son semejantes respecto al eje, cada cara lateral es
    plana (contiene el vértice del cono); se construye con caras planas en vez
    de un loft para que las aristas resultantes sean rectas y admitan chaflanes."""
    a, b = slot_polygon(z0), slot_polygon(z1)
    n = len(a)
    faces = [cq.Face.makeFromWires(cq.Wire.makePolygon(a, close=True)),
             cq.Face.makeFromWires(cq.Wire.makePolygon(b, close=True))]
    for i in range(n):
        j = (i + 1) % n
        faces.append(cq.Face.makeFromWires(cq.Wire.makePolygon([a[i], a[j], b[j], b[i]], close=True)))
    solid = cq.Solid.makeSolid(cq.Shell.makeShell(faces)).fix()
    if solid.Volume() < 0:
        solid = cq.Solid(solid.wrapped.Reversed())
    return solid


def edges_at(shape, z, r_min, r_max, tol=1e-3):
    """Aristas contenidas en el plano z cuya distancia al eje está en [r_min, r_max]."""
    out = []
    for e in shape.Edges():
        bb = e.BoundingBox()
        if abs(bb.zmin - z) > tol or abs(bb.zmax - z) > tol:
            continue
        rs = [math.hypot(p.x, p.y) for p in (e.startPoint(), e.endPoint(), e.positionAt(0.5))]
        if all(r_min - tol <= r <= r_max + tol for r in rs):
            out.append(e)
    return out


def check_parameters():
    errors = []
    k_small = SMALL_DIAMETER / LARGE_DIAMETER
    yoke_small = (LARGE_DIAMETER / 2 - SLOT_DEPTH) * k_small - SHAFT_HOLE_DIAMETER / 2
    if yoke_small < MIN_WALL:
        errors.append('Corona bajo las ranuras en el extremo menor = %.2f mm' % yoke_small)
    if not SLOT_OPENING / 2 < flank_point(LARGE_DIAMETER / 2 - SHOE_ROOT_THICKNESS, 1)[1]:
        errors.append('SLOT_OPENING demasiado grande para la zapata')
    if SHOE_TIP_THICKNESS * k_small <= 2 * BOTTOM_CHAMFER:
        errors.append('Zapata demasiado fina para BOTTOM_CHAMFER en el extremo menor')
    if flank_point(LARGE_DIAMETER / 2 - SLOT_DEPTH, 1)[1] <= 0.5:
        errors.append('Los dientes se tocan en el fondo de ranura (reduce TOOTH_WIDTH o NUM_SLOTS)')
    hub_clear = (LARGE_DIAMETER / 2 - SLOT_DEPTH) - HUB_DIAMETER / 2
    if hub_clear < 3:
        errors.append('El cubo tapa el fondo de las ranuras (holgura %.1f mm)' % hub_clear)
    if errors:
        raise ValueError('\n'.join(errors))


def build():
    check_parameters()
    h = ROTOR_HEIGHT
    r_hole = SHAFT_HOLE_DIAMETER / 2
    total_h = h + HUB_LENGTH

    # 1. Núcleo troncocónico (menor abajo, mayor arriba)
    core = cq.Solid.makeCone(SMALL_DIAMETER / 2, LARGE_DIAMETER / 2, h)

    # 2. Ranuras: cortador cónico hacia el vértice del cono (sobrepasa 1 mm cada cara)
    cutter = slot_cutter(-1.0, h + 1.0)
    for i in range(NUM_SLOTS):
        core = core.cut(cutter.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), i * 360.0 / NUM_SLOTS))

    # 3. Cubo en el extremo mayor
    hub = cq.Solid.makeCylinder(HUB_DIAMETER / 2, HUB_LENGTH, cq.Vector(0, 0, h))
    rotor = core.fuse(hub).clean()

    # 4. Agujero del eje, pasante y concéntrico
    bore = cq.Solid.makeCylinder(r_hole, total_h + 2, cq.Vector(0, 0, -1))
    rotor = rotor.cut(bore).clean()

    # 5. Acabados de aristas
    rotor = rotor.fillet(TOP_FILLET, edges_at(rotor, h, HUB_DIAMETER / 2 + 0.5, LARGE_DIAMETER))
    rotor = rotor.fillet(HUB_FILLET, edges_at(rotor, total_h, HUB_DIAMETER / 2 - 0.01, HUB_DIAMETER / 2 + 0.01))
    rotor = rotor.chamfer(BOTTOM_CHAMFER, None, edges_at(rotor, 0, r_hole + 0.5, LARGE_DIAMETER))
    entry = edges_at(rotor, 0, r_hole, r_hole) + edges_at(rotor, total_h, r_hole, r_hole)
    rotor = rotor.chamfer(SHAFT_ENTRY_CHAMFER, None, entry)

    # 6. Prisionero M3 radial en el cubo (única geometría no axisimétrica)
    screw = cq.Solid.makeCylinder(SET_SCREW_HOLE_DIAMETER / 2, HUB_DIAMETER / 2 + 1,
                                  cq.Vector(0, 0, h + HUB_LENGTH / 2), cq.Vector(1, 0, 0))
    rotor = rotor.cut(screw).clean()
    return rotor


def verify(rotor):
    solids = rotor.Solids()
    assert len(solids) == 1, 'Se esperaba 1 sólido, hay %d' % len(solids)
    assert rotor.isValid(), 'Geometría BRep no válida'
    shells = solids[0].Shells()
    assert len(shells) == 1, 'Hay huecos internos o varias carcasas'
    vol = rotor.Volume()
    com = cq.Shape.centerOfMass(rotor)
    k_small = SMALL_DIAMETER / LARGE_DIAMETER
    return {
        'volumen_cm3': vol / 1000,
        'masa_PLA_g': vol / 1000 * 1.24,
        'desplazamiento_CdM_mm': math.hypot(com.x, com.y),
        'corona_min_mm': (LARGE_DIAMETER / 2 - SLOT_DEPTH) * k_small - SHAFT_HOLE_DIAMETER / 2,
        'boca_ranura_extremo_menor_mm': SLOT_OPENING * k_small,
        'zapata_min_mm': SHOE_TIP_THICKNESS * k_small,
        'altura_total_mm': ROTOR_HEIGHT + HUB_LENGTH,
    }


def check_mesh(path):
    """Comprueba que el STL sea estanco (requiere trimesh; devuelve -1 si no está)."""
    try:
        import trimesh
    except ImportError:
        return -1
    mesh = trimesh.load(path)
    if not (mesh.is_watertight and mesh.is_winding_consistent and mesh.body_count == 1):
        raise RuntimeError('La malla exportada no es estanca: ' + path)
    return 1


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    rotor = build()
    info = verify(rotor)
    base = os.path.join(OUT_DIR, NAME)
    cq.exporters.export(rotor, base + '.step')
    # angularTolerance 0.05: con valores mayores la malla de los redondeos en las
    # esquinas del fondo de ranura deja aristas no-manifold
    cq.exporters.export(rotor, base + '.stl', tolerance=0.01, angularTolerance=0.05)
    cq.exporters.export(rotor, base + '.3mf', tolerance=0.01, angularTolerance=0.05)
    info['malla_estanca'] = check_mesh(base + '.stl')
    for key, value in info.items():
        print('%-30s %.3f' % (key, value))
    return rotor


if __name__ == '__main__':
    main()

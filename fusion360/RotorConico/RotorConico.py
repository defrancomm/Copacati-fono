# -*- coding: utf-8 -*-
"""
RotorConico.py  -  Script para Autodesk Fusion (API Python)

Construye un rotor troncocónico paramétrico, concéntrico con el eje Z y
preparado para impresión FDM:

    Rotor_Profile        sketch con medio perfil longitudinal (plano XZ)
    Rotor_Revolve        revolución 360° alrededor del eje Z -> cuerpo sólido
    Shaft_Hole_Sketch    círculo centrado en el origen (plano XY)
    Shaft_Hole           corte por extrusión: alojamiento pasante del eje
    Shaft_Entry_Chamfer  chaflán en las dos entradas del agujero
    Outer_Chamfer        chaflán arista exterior de la base (apoyo en cama)
    Outer_Fillet         redondeo arista exterior de la cara superior
    SetScrew_Sketch      medio perfil escalonado del agujero radial (plano XZ)
    SetScrew_Hole        revolución-corte: paso M3 + alojamiento de inserto
                         térmico + pozo de acceso

Todas las cotas de los sketches y features están ligadas a parámetros de
usuario, por lo que el modelo se modifica desde Modificar > Cambiar parámetros.

Convenciones: la cara de diámetro mayor está en z = 0 (cara de apoyo en la
cama de impresión) y la de diámetro menor en z = rotorHeight.
"""

import math
import os
import traceback

import adsk.core
import adsk.fusion

# ---------------------------------------------------------------------------
# Opciones del script
# ---------------------------------------------------------------------------
CREATE_NEW_DOCUMENT = True      # False -> construye dentro del diseño activo
EXPORT_FOLDER = ''              # p.ej. r'C:\Users\yo\Desktop' para exportar STL y 3MF automáticamente
COMPONENT_NAME = 'Rotor_Conico'
BODY_NAME = 'Rotor_Conico_Body'
MIN_WALL_MM = 2.0               # espesor de pared mínimo aceptado (FDM, boquilla 0.4 mm)
TOL_CM = 1e-4                   # tolerancia geométrica (la API trabaja en cm)

# Parámetros de usuario: (nombre, expresión, unidades, comentario).
# El orden importa: un parámetro solo puede referenciar a los anteriores.
USER_PARAMETERS = [
    # --- Geometría general ---
    ('rotorHeight', '60 mm', 'mm', 'Altura total del rotor'),
    ('largeDiameter', '50 mm', 'mm', 'Diámetro mayor (cara inferior, z = 0)'),
    ('smallDiameter', '30 mm', 'mm', 'Diámetro menor (cara superior)'),
    # --- Eje ---
    ('shaftDiameter', '8 mm', 'mm', 'Diámetro nominal del eje del motor'),
    ('shaftClearance', '0.2 mm', 'mm', 'Holgura diametral entre agujero y eje'),
    ('shaftHoleDiameter', 'shaftDiameter + shaftClearance', 'mm', 'Diámetro del alojamiento del eje'),
    ('shaftHoleDepth', 'rotorHeight', 'mm', 'Profundidad del alojamiento (= rotorHeight -> pasante)'),
    ('shaftEntryChamfer', '0.5 mm', 'mm', 'Chaflán de entrada del eje (ambas caras)'),
    # --- Acabado de aristas exteriores ---
    ('chamferSize', '1 mm', 'mm', 'Chaflán arista exterior inferior (evita pata de elefante)'),
    ('filletRadius', '0.8 mm', 'mm', 'Redondeo arista exterior superior'),
    # --- Prisionero radial M3 con inserto térmico ---
    ('setScrewSize', '3 mm', 'mm', 'Métrica nominal del prisionero (M3)'),
    ('setScrewPassDiameter', 'setScrewSize + 0.4 mm', 'mm', 'Paso libre del prisionero entre inserto y eje'),
    ('setScrewOffset', '10 mm', 'mm', 'Distancia axial desde la cara mayor al eje del agujero radial'),
    ('insertHoleDiameter', '4 mm', 'mm', 'Agujero para inserto térmico M3 (tipo M3x5.7, Ø ext. ~4.6 mm)'),
    ('insertDepth', '6 mm', 'mm', 'Profundidad del alojamiento del inserto'),
    ('insertSeatWall', '2 mm', 'mm', 'Material entre el alojamiento del eje y el fondo del inserto'),
    ('setScrewAccessDiameter', '7 mm', 'mm', 'Pozo de acceso para punta de soldador y llave Allen'),
]


# ---------------------------------------------------------------------------
# Utilidades generales
# ---------------------------------------------------------------------------
def create_or_update_parameters(design):
    """Crea los parámetros de usuario (o actualiza su expresión si ya existen)."""
    params = design.userParameters
    for name, expression, units, comment in USER_PARAMETERS:
        param = params.itemByName(name)
        if param:
            param.expression = expression
            param.comment = comment
        else:
            param = params.add(name, adsk.core.ValueInput.createByString(expression), units, comment)
        param.isFavorite = True


def pval(design, name):
    """Valor de un parámetro de usuario en unidades internas (cm)."""
    return design.userParameters.itemByName(name).value


def sketch_point(sketch, r_cm, z_cm):
    """Convierte un punto del modelo (X = radio, Z = altura, Y = 0) al espacio del sketch."""
    return sketch.modelToSketchSpace(adsk.core.Point3D.create(r_cm, 0, z_cm))


class SketchAxes:
    """Resuelve qué dirección del sketch (horizontal/vertical) corresponde al radio
    (X del modelo) y cuál a la altura (Z del modelo). Así no dependemos de la
    orientación interna del plano XZ en Fusion."""

    def __init__(self, sketch):
        o = sketch.modelToSketchSpace(adsk.core.Point3D.create(0, 0, 0))
        x = sketch.modelToSketchSpace(adsk.core.Point3D.create(1, 0, 0))
        self._radial_is_horizontal = abs(x.x - o.x) > abs(x.y - o.y)
        self._constraints = sketch.geometricConstraints
        h = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
        v = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation
        self.radial_dim = h if self._radial_is_horizontal else v
        self.axial_dim = v if self._radial_is_horizontal else h

    def radial(self, line):
        """Restringe una línea a ser perpendicular al eje Z (dirección radial)."""
        if self._radial_is_horizontal:
            self._constraints.addHorizontal(line)
        else:
            self._constraints.addVertical(line)

    def axial(self, line):
        """Restringe una línea a ser paralela al eje Z."""
        if self._radial_is_horizontal:
            self._constraints.addVertical(line)
        else:
            self._constraints.addHorizontal(line)


def add_dimension(sketch, p1, p2, orientation, expression):
    """Cota de distancia entre dos puntos de sketch ligada a una expresión paramétrica."""
    g1, g2 = p1.geometry, p2.geometry
    text_point = adsk.core.Point3D.create((g1.x + g2.x) / 2 + 0.5, (g1.y + g2.y) / 2 + 0.5, 0)
    dim = sketch.sketchDimensions.addDistanceDimension(p1, p2, orientation, text_point)
    dim.parameter.expression = expression
    return dim


def circular_edges(body, radius_cm, z_cm):
    """Aristas circulares del cuerpo con el radio y la altura indicados."""
    edges = adsk.core.ObjectCollection.create()
    for edge in body.edges:
        geom = edge.geometry
        if geom.objectType != adsk.core.Circle3D.classType():
            continue
        if abs(geom.radius - radius_cm) < TOL_CM and abs(geom.center.z - z_cm) < TOL_CM:
            edges.add(edge)
    return edges


def outer_radius_at(design, z_cm):
    """Radio exterior del cono a la altura z (sin chaflanes)."""
    r1 = pval(design, 'largeDiameter') / 2
    r2 = pval(design, 'smallDiameter') / 2
    h = pval(design, 'rotorHeight')
    return r1 - (r1 - r2) * z_cm / h


# ---------------------------------------------------------------------------
# Comprobación previa de parámetros (evita geometría imposible)
# ---------------------------------------------------------------------------
def check_parameters(design):
    v = lambda n: pval(design, n) * 10.0   # en mm
    errors = []

    if v('largeDiameter') <= v('smallDiameter'):
        errors.append('largeDiameter debe ser mayor que smallDiameter.')
    if v('shaftClearance') < 0:
        errors.append('shaftClearance no puede ser negativa.')
    if v('shaftHoleDepth') < v('rotorHeight') - 1e-6:
        errors.append('shaftHoleDepth < rotorHeight: el agujero del eje no sería pasante.')

    wall_small = (v('smallDiameter') - v('shaftHoleDiameter')) / 2
    if wall_small - max(v('filletRadius'), v('shaftEntryChamfer')) < MIN_WALL_MM:
        errors.append('Pared radial en el extremo menor insuficiente ({:.2f} mm).'.format(wall_small))
    if v('chamferSize') <= 0 or v('filletRadius') <= 0 or v('shaftEntryChamfer') <= 0:
        errors.append('chamferSize, filletRadius y shaftEntryChamfer deben ser > 0.')
    if v('shaftEntryChamfer') + v('chamferSize') >= (v('largeDiameter') - v('shaftHoleDiameter')) / 2:
        errors.append('Los chaflanes de la cara inferior se solapan.')
    if v('shaftEntryChamfer') + v('filletRadius') >= wall_small:
        errors.append('El chaflán de entrada y el redondeo superior se solapan.')

    # Agujero radial
    if not v('setScrewPassDiameter') < v('insertHoleDiameter') < v('setScrewAccessDiameter'):
        errors.append('Debe cumplirse setScrewPassDiameter < insertHoleDiameter < setScrewAccessDiameter.')
    z_lo = v('setScrewOffset') - v('setScrewAccessDiameter') / 2
    z_hi = v('setScrewOffset') + v('setScrewAccessDiameter') / 2
    if z_lo < v('chamferSize') + 1.0 or z_hi > v('rotorHeight') - v('filletRadius') - 1.0:
        errors.append('setScrewOffset sitúa el pozo de acceso demasiado cerca de una cara plana.')
    r_insert_end = v('shaftHoleDiameter') / 2 + v('insertSeatWall') + v('insertDepth')
    r_outer_min = outer_radius_at(design, z_hi / 10.0) * 10.0
    if r_insert_end > r_outer_min - 0.5:
        errors.append('El alojamiento del inserto sobresale del rotor; reduce insertDepth/insertSeatWall '
                      'o acerca setScrewOffset a la cara mayor.')
    if v('insertSeatWall') < 1.0:
        errors.append('insertSeatWall debe ser >= 1 mm.')
    return errors


# ---------------------------------------------------------------------------
# Construcción de features
# ---------------------------------------------------------------------------
def build_rotor_body(design, comp):
    """Rotor_Profile + Rotor_Revolve: tronco de cono sólido alrededor del eje Z."""
    sketch = comp.sketches.add(comp.xZConstructionPlane)   # plano vertical que contiene Z
    sketch.name = 'Rotor_Profile'
    axes = SketchAxes(sketch)
    lines = sketch.sketchCurves.sketchLines

    r1 = pval(design, 'largeDiameter') / 2
    r2 = pval(design, 'smallDiameter') / 2
    h = pval(design, 'rotorHeight')
    origin = sketch.originPoint

    # Medio perfil: base (radial) -> generatriz cónica -> cara superior (radial) -> eje (axial)
    base = lines.addByTwoPoints(origin, sketch_point(sketch, r1, 0))
    cone = lines.addByTwoPoints(base.endSketchPoint, sketch_point(sketch, r2, h))
    top = lines.addByTwoPoints(cone.endSketchPoint, sketch_point(sketch, 0, h))
    axis = lines.addByTwoPoints(top.endSketchPoint, origin)

    axes.radial(base)
    axes.radial(top)
    axes.axial(axis)        # el lado interior coincide con el eje Z -> sólido sin hueco central

    add_dimension(sketch, origin, base.endSketchPoint, axes.radial_dim, 'largeDiameter / 2')
    add_dimension(sketch, top.startSketchPoint, top.endSketchPoint, axes.radial_dim, 'smallDiameter / 2')
    add_dimension(sketch, origin, top.endSketchPoint, axes.axial_dim, 'rotorHeight')

    if sketch.profiles.count != 1:
        raise RuntimeError('Rotor_Profile debería tener exactamente un perfil cerrado.')

    # Revolución de 360° alrededor del eje Z del componente
    revolves = comp.features.revolveFeatures
    rev_input = revolves.createInput(sketch.profiles.item(0), comp.zConstructionAxis,
                                     adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    rev_input.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
    revolve = revolves.add(rev_input)
    revolve.name = 'Rotor_Revolve'

    body = revolve.bodies.item(0)
    body.name = BODY_NAME
    return body


def build_shaft_hole(design, comp, body):
    """Shaft_Hole_Sketch + Shaft_Hole: agujero axial pasante y concéntrico."""
    sketch = comp.sketches.add(comp.xYConstructionPlane)   # cara mayor, z = 0
    sketch.name = 'Shaft_Hole_Sketch'
    r = pval(design, 'shaftHoleDiameter') / 2
    # Centro = origen del sketch = eje Z -> concentricidad exacta por construcción
    circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(sketch.originPoint, r)
    dim = sketch.sketchDimensions.addDiameterDimension(circle, adsk.core.Point3D.create(r * 1.5, r * 1.5, 0))
    dim.parameter.expression = 'shaftHoleDiameter'

    extrudes = comp.features.extrudeFeatures
    ext_input = extrudes.createInput(sketch.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
    extent = adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString('shaftHoleDepth'))
    ext_input.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)   # hacia +Z
    ext_input.participantBodies = [body]
    cut = extrudes.add(ext_input)
    cut.name = 'Shaft_Hole'


def build_edge_treatments(design, comp, body):
    """Shaft_Entry_Chamfer, Outer_Chamfer y Outer_Fillet."""
    h = pval(design, 'rotorHeight')
    r_hole = pval(design, 'shaftHoleDiameter') / 2
    r1 = pval(design, 'largeDiameter') / 2
    r2 = pval(design, 'smallDiameter') / 2
    chamfers = comp.features.chamferFeatures

    # --- Chaflán en las dos entradas del agujero del eje ---
    entry_edges = circular_edges(body, r_hole, 0)
    for edge in circular_edges(body, r_hole, h):
        entry_edges.add(edge)
    if entry_edges.count != 2:
        raise RuntimeError('No se encontraron las 2 aristas de entrada del eje ({}).'.format(entry_edges.count))
    ch_input = chamfers.createInput2()
    ch_input.chamferEdgeSets.addEqualDistanceChamferEdgeSet(
        entry_edges, adsk.core.ValueInput.createByString('shaftEntryChamfer'), False)
    chamfers.add(ch_input).name = 'Shaft_Entry_Chamfer'

    # --- Chaflán exterior inferior: en la cara de apoyo un chaflán imprime mejor
    #     que un redondeo (no crea voladizo y compensa la "pata de elefante") ---
    bottom_edges = circular_edges(body, r1, 0)
    if bottom_edges.count != 1:
        raise RuntimeError('No se encontró la arista exterior inferior.')
    ch_input = chamfers.createInput2()
    ch_input.chamferEdgeSets.addEqualDistanceChamferEdgeSet(
        bottom_edges, adsk.core.ValueInput.createByString('chamferSize'), False)
    chamfers.add(ch_input).name = 'Outer_Chamfer'

    # --- Redondeo exterior superior ---
    top_edges = circular_edges(body, r2, h)
    if top_edges.count != 1:
        raise RuntimeError('No se encontró la arista exterior superior.')
    fillets = comp.features.filletFeatures
    fi_input = fillets.createInput()
    fi_input.edgeSetInputs.addConstantRadiusEdgeSet(
        top_edges, adsk.core.ValueInput.createByString('filletRadius'), False)
    fillets.add(fi_input).name = 'Outer_Fillet'


def build_set_screw_hole(design, comp, body):
    """SetScrew_Sketch + SetScrew_Hole: agujero radial escalonado según +X.

    Desde el eje hacia fuera:
      Ø setScrewPassDiameter   del alojamiento del eje hasta el fondo del inserto
      Ø insertHoleDiameter     alojamiento del inserto térmico (insertDepth)
      Ø setScrewAccessDiameter pozo de acceso hasta salir de la superficie cónica
    Se dibuja medio perfil y se revoluciona 360° alrededor de su propio eje.
    """
    sketch = comp.sketches.add(comp.xZConstructionPlane)
    sketch.name = 'SetScrew_Sketch'
    axes = SketchAxes(sketch)
    lines = sketch.sketchCurves.sketchLines

    zs = pval(design, 'setScrewOffset')
    rp = pval(design, 'setScrewPassDiameter') / 2
    ri = pval(design, 'insertHoleDiameter') / 2
    ra = pval(design, 'setScrewAccessDiameter') / 2
    r_seat = pval(design, 'shaftHoleDiameter') / 2 + pval(design, 'insertSeatWall')
    r_insert = r_seat + pval(design, 'insertDepth')
    r_end = pval(design, 'largeDiameter') / 2 + 0.2     # 2 mm más allá de la superficie

    # Línea de construcción que fija la altura del eje del agujero radial
    ref = lines.addByTwoPoints(sketch.originPoint, sketch_point(sketch, 0, zs))
    ref.isConstruction = True
    axes.axial(ref)
    add_dimension(sketch, sketch.originPoint, ref.endSketchPoint, axes.axial_dim, 'setScrewOffset')

    # Medio perfil escalonado (cerrado)
    p0 = ref.endSketchPoint
    l1 = lines.addByTwoPoints(p0, sketch_point(sketch, 0, zs + rp))
    l2 = lines.addByTwoPoints(l1.endSketchPoint, sketch_point(sketch, r_seat, zs + rp))
    l3 = lines.addByTwoPoints(l2.endSketchPoint, sketch_point(sketch, r_seat, zs + ri))
    l4 = lines.addByTwoPoints(l3.endSketchPoint, sketch_point(sketch, r_insert, zs + ri))
    l5 = lines.addByTwoPoints(l4.endSketchPoint, sketch_point(sketch, r_insert, zs + ra))
    l6 = lines.addByTwoPoints(l5.endSketchPoint, sketch_point(sketch, r_end, zs + ra))
    l7 = lines.addByTwoPoints(l6.endSketchPoint, sketch_point(sketch, r_end, zs))
    hole_axis = lines.addByTwoPoints(l7.endSketchPoint, p0)     # eje radial, perpendicular a Z

    for line in (l1, l3, l5, l7):
        axes.axial(line)
    for line in (l2, l4, l6, hole_axis):
        axes.radial(line)

    add_dimension(sketch, l1.startSketchPoint, l1.endSketchPoint, axes.axial_dim, 'setScrewPassDiameter / 2')
    add_dimension(sketch, l2.startSketchPoint, l2.endSketchPoint, axes.radial_dim,
                  'shaftHoleDiameter / 2 + insertSeatWall')
    add_dimension(sketch, l3.startSketchPoint, l3.endSketchPoint, axes.axial_dim,
                  '(insertHoleDiameter - setScrewPassDiameter) / 2')
    add_dimension(sketch, l4.startSketchPoint, l4.endSketchPoint, axes.radial_dim, 'insertDepth')
    add_dimension(sketch, l5.startSketchPoint, l5.endSketchPoint, axes.axial_dim,
                  '(setScrewAccessDiameter - insertHoleDiameter) / 2')
    add_dimension(sketch, hole_axis.startSketchPoint, hole_axis.endSketchPoint, axes.radial_dim,
                  'largeDiameter / 2 + 2 mm')

    if sketch.profiles.count != 1:
        raise RuntimeError('SetScrew_Sketch debería tener exactamente un perfil cerrado.')

    revolves = comp.features.revolveFeatures
    rev_input = revolves.createInput(sketch.profiles.item(0), hole_axis,
                                     adsk.fusion.FeatureOperations.CutFeatureOperation)
    rev_input.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
    rev_input.participantBodies = [body]
    revolves.add(rev_input).name = 'SetScrew_Hole'


# ---------------------------------------------------------------------------
# Verificación final del modelo
# ---------------------------------------------------------------------------
def verify_model(design, comp):
    errors, report = [], []

    # Salud de todas las operaciones del timeline
    timeline = design.timeline
    for i in range(timeline.count):
        item = timeline.item(i)
        if item.healthState != adsk.fusion.FeatureHealthStates.HealthyFeatureHealthState:
            errors.append('Timeline "{}": {}'.format(item.name, item.errorOrWarningMessage))

    # Un único cuerpo sólido, cerrado, de un solo lump y una sola shell exterior
    if comp.bRepBodies.count != 1:
        errors.append('El componente contiene {} cuerpos (se esperaba 1).'.format(comp.bRepBodies.count))
        return errors, report, None
    body = comp.bRepBodies.item(0)
    if not body.isSolid:
        errors.append('El cuerpo no es sólido (cuerpo abierto/superficie).')
    if body.lumps.count != 1:
        errors.append('El cuerpo tiene {} lumps (debe ser 1 pieza continua).'.format(body.lumps.count))
    for shell in body.shells:
        if not shell.isClosed:
            errors.append('Hay una shell abierta en el cuerpo.')
        if shell.isVoid:
            errors.append('El cuerpo contiene un hueco interno cerrado.')

    # Concentricidad del alojamiento del eje
    r_hole = pval(design, 'shaftHoleDiameter') / 2
    bore_faces = 0
    for face in body.faces:
        geom = face.geometry
        if geom.objectType != adsk.core.Cylinder.classType() or abs(geom.radius - r_hole) > TOL_CM:
            continue
        axis = geom.axis.copy()
        axis.normalize()
        if abs(abs(axis.z) - 1.0) > 1e-9:
            continue
        bore_faces += 1
        offset = math.hypot(geom.origin.x, geom.origin.y)
        if offset > 1e-7:
            errors.append('Agujero del eje descentrado {:.6f} mm.'.format(offset * 10))
    if bore_faces == 0:
        errors.append('No se encontró la cara cilíndrica del alojamiento del eje.')

    # Espesores de pared
    wall_small = (pval(design, 'smallDiameter') - pval(design, 'shaftHoleDiameter')) / 2 * 10
    report.append('Pared mínima extremo menor: {:.2f} mm'.format(wall_small))
    zs_top = pval(design, 'setScrewOffset') + pval(design, 'setScrewAccessDiameter') / 2
    r_insert = pval(design, 'shaftHoleDiameter') / 2 + pval(design, 'insertSeatWall') + pval(design, 'insertDepth')
    report.append('Profundidad pozo de acceso: {:.2f} mm'.format((outer_radius_at(design, zs_top) - r_insert) * 10))

    # Propiedades físicas y equilibrado
    props = body.physicalProperties
    com = props.centerOfMass
    report.append('Volumen: {:.2f} cm³'.format(props.volume))
    report.append('Desplazamiento radial del CdM: {:.3f} mm (solo por el agujero radial)'.format(
        math.hypot(com.x, com.y) * 10))
    if props.volume <= 0:
        errors.append('Volumen nulo o negativo.')
    return errors, report, body


def export_for_printing(design, body, folder):
    """Exporta el cuerpo a STL (malla fina) y 3MF."""
    exporter = design.exportManager
    stl_path = os.path.join(folder, COMPONENT_NAME + '.stl')
    stl_options = exporter.createSTLExportOptions(body, stl_path)
    stl_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
    exporter.execute(stl_options)
    tmf_path = os.path.join(folder, COMPONENT_NAME + '.3mf')
    exporter.execute(exporter.createC3MFExportOptions(body, tmf_path))
    return [stl_path, tmf_path]


# ---------------------------------------------------------------------------
# Punto de entrada
# ---------------------------------------------------------------------------
def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        # 1. Diseño
        if CREATE_NEW_DOCUMENT or not adsk.fusion.Design.cast(app.activeProduct):
            app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        root = design.rootComponent

        # 2. Componente Rotor_Conico (transformación identidad: su eje Z = eje Z global)
        occurrence = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        comp = occurrence.component
        comp.name = COMPONENT_NAME

        # 3. Parámetros de usuario + comprobación de coherencia
        create_or_update_parameters(design)
        param_errors = check_parameters(design)
        if param_errors:
            ui.messageBox('Parámetros no válidos:\n\n- ' + '\n- '.join(param_errors), 'Rotor_Conico')
            return

        # 4-6. Perfil + revolución
        body = build_rotor_body(design, comp)
        # 7. Agujero axial
        build_shaft_hole(design, comp, body)
        # 8. Chaflanes y redondeos
        build_edge_treatments(design, comp, body)
        # 9. Agujero radial del prisionero M3 (inserto térmico)
        build_set_screw_hole(design, comp, body)

        # 10. Verificación
        errors, report, body = verify_model(design, comp)

        # 11. Vista
        viewport = app.activeViewport
        camera = viewport.camera
        camera.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
        camera.isFitView = True
        viewport.camera = camera
        viewport.fit()

        exported = []
        if EXPORT_FOLDER and body and not errors:
            exported = export_for_printing(design, body, EXPORT_FOLDER)

        message = 'Rotor_Conico generado.\n\n' + '\n'.join(report)
        if exported:
            message += '\n\nExportado:\n' + '\n'.join(exported)
        if errors:
            message += '\n\nPROBLEMAS DETECTADOS:\n- ' + '\n- '.join(errors)
        else:
            message += '\n\nVerificación OK: un único cuerpo sólido, cerrado y concéntrico.'
        ui.messageBox(message, 'Rotor_Conico')

    except Exception:
        ui.messageBox('Error al generar el rotor:\n{}'.format(traceback.format_exc()), 'Rotor_Conico')

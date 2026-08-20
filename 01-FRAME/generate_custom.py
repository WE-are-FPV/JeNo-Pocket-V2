#!/usr/bin/env python3
"""Build a standalone X-Core bottom plate with the V2 Tank center cutouts."""

from pathlib import Path
import math
import ezdxf
from ezdxf import bbox
from ezdxf.addons import Importer
from ezdxf.path import make_path
from ezdxf.transform import inplace
from ezdxf.math import Matrix44
from reportlab.lib.pagesizes import A4, landscape
from reportlab.pdfgen.canvas import Canvas
from shapely.geometry import LineString
from shapely.ops import polygonize, unary_union
import trimesh


HERE = Path(__file__).resolve().parent
SOURCE = HERE / "JeNoPocket_V2_ALL_VERSIONS_2.1.0.dxf"
STEM = "JeNoPocket_V2_Custom"
TOP_STEM = "JeNoPocket_V2_Custom_Top_Plate"

# Centers measured from the source drawing. The FC pattern is a 25.5 mm square
# rotated 45 degrees, so its horizontal/vertical diagonals are 25.5*sqrt(2).
XCORE_CENTER = (116.457, -169.963)
TANK_CENTER = (115.885, -507.515)
PATTERN_HALF_DIAGONAL = 18.2  # 25.5/sqrt(2), with selection tolerance
FRAME_WINDOW = (55.0, -230.0, 178.0, -110.0)
TOP_PLATE_WINDOW = (290.0, -225.0, 335.0, -115.0)
M2_CUT_RADIUS = 1.0
NEW_PATTERN_HALF_SPACING = 10.0
FRONT_FEATURE_CENTER_SPACING = 21.0
ORIGINAL_FRONT_FEATURE_CENTER_SPACING = 16.0
FRONT_SLOT_SHIFT = (
    FRONT_FEATURE_CENTER_SPACING - ORIGINAL_FRONT_FEATURE_CENTER_SPACING
) / 2.0
TOP_FRONT_TRIM_HALF_WIDTH = 14.4
TOP_FRONT_TRIM_FILLET_RADIUS = 1.0
TOP_FRONT_UPPER_FILLET_RADIUS = 0.75


def bounds(entity):
    box = bbox.extents([entity], fast=True)
    return box if box.has_data else None


def inside_box(entity, cx, cy, half):
    box = bounds(entity)
    return bool(
        box
        and box.extmin.x >= cx - half
        and box.extmax.x <= cx + half
        and box.extmin.y >= cy - half
        and box.extmax.y <= cy + half
    )


def in_frame_window(entity):
    box = bounds(entity)
    x0, y0, x1, y1 = FRAME_WINDOW
    return bool(
        box
        and box.extmin.x >= x0
        and box.extmax.x <= x1
        and box.extmin.y >= y0
        and box.extmax.y <= y1
    )


def in_window(entity, window):
    box = bounds(entity)
    x0, y0, x1, y1 = window
    return bool(
        box
        and box.extmin.x >= x0
        and box.extmax.x <= x1
        and box.extmin.y >= y0
        and box.extmax.y <= y1
    )


def translate_front_keyholes(doc, center_x, top_y):
    """Move the two front keyholes to 21 mm center-to-center spacing."""
    keyholes = []
    for entity in doc.modelspace().query("LWPOLYLINE"):
        if entity.dxf.layer != "Cut" or len(entity) != 8:
            continue
        box = bounds(entity)
        if not box or box.extmax.y < top_y - 17.0:
            continue
        if not (3.5 <= box.size.x <= 3.9 and 4.9 <= box.size.y <= 6.1):
            continue
        keyholes.append(entity)

    for entity in keyholes:
        box = bounds(entity)
        sign = -1.0 if box.center.x < center_x else 1.0
        target_x = center_x + sign * FRONT_FEATURE_CENTER_SPACING / 2.0
        inplace([entity], Matrix44.translate(target_x - box.center.x, 0.0, 0.0))
    return len(keyholes)


def widen_bottom_front_profile(doc, center_x):
    """Widen the camera end and replace its tapered neck with straight sides."""
    outlines = []
    for entity in doc.modelspace().query("LWPOLYLINE"):
        box = bounds(entity)
        if (
            entity.dxf.layer == "Calque1"
            and len(entity) == 69
            and box
            and box.size.x > 50.0
            and box.size.y > 90.0
        ):
            outlines.append(entity)

    for outline in outlines:
        points = [list(point) for point in outline.get_points(format="xyseb")]
        sign = -1.0 if max(point[0] for point in points) <= center_x + 0.01 else 1.0
        # The source outline has a few microns of left/right drafting
        # asymmetry, so derive the shift from the notch's actual center and
        # land that center exactly at +/-10.5 mm.
        notch_center_x = (points[61][0] + points[64][0]) / 2.0
        profile_shift = (
            center_x + sign * FRONT_FEATURE_CENTER_SPACING / 2.0 - notch_center_x
        )
        side_x = points[57][0] + profile_shift

        # Start the straight wall at the arm transition and carry it through
        # the upper corner, eliminating the original inward taper.
        for index in range(52, 58):
            points[index][0] = side_x
        for index in range(52, 57):
            points[index][4] = 0.0

        # Move the rounded corner, notch, and notch inner end as one group.
        for index in range(58, 66):
            points[index][0] += profile_shift
        outline.set_points(points, format="xyseb")
    return len(outlines)


def widen_top_plate_front_profile(doc):
    """Widen the top plate shoulders and move its two notches outward."""
    candidates = []
    for entity in doc.modelspace().query("LWPOLYLINE"):
        box = bounds(entity)
        if entity.dxf.layer == "Cut" and len(entity) == 67 and box and box.size.y > 60.0:
            candidates.append(entity)
    if len(candidates) != 1:
        raise RuntimeError(f"Expected one top-plate outline, found {len(candidates)}")

    outline = candidates[0]
    points = [list(point) for point in outline.get_points(format="xyseb")]

    # Move each notch and upper corner the full 2.5 mm. Blend the widening
    # through the short existing shoulder curves to keep the profile smooth.
    left_start_y = points[14][1]
    left_full_y = points[17][1]
    for index in range(14, 18):
        blend = (points[index][1] - left_start_y) / (left_full_y - left_start_y)
        points[index][0] -= FRONT_SLOT_SHIFT * blend
    for index in range(18, 29):
        points[index][0] -= FRONT_SLOT_SHIFT

    for index in range(40, 51):
        points[index][0] += FRONT_SLOT_SHIFT
    right_full_y = points[51][1]
    right_end_y = points[54][1]
    for index in range(51, 55):
        blend = (points[index][1] - right_end_y) / (right_full_y - right_end_y)
        points[index][0] += FRONT_SLOT_SHIFT * blend

    outline.set_points(points, format="xyseb")
    return 1


def trim_top_plate_front_sides(doc, center_x):
    """Trim the widened top-plate shoulders to symmetric vertical sides."""
    candidates = [
        entity
        for entity in doc.modelspace().query("LWPOLYLINE")
        if entity.dxf.layer == "Cut" and len(entity) == 67
    ]
    if len(candidates) != 1:
        raise RuntimeError(f"Expected one widened top-plate outline, found {len(candidates)}")

    outline = candidates[0]
    points = [list(point) for point in outline.get_points(format="xyseb")]

    def y_at_x(first, second, target_x):
        ratio = (target_x - first[0]) / (second[0] - first[0])
        return first[1] + ratio * (second[1] - first[1])

    left_x = center_x - TOP_FRONT_TRIM_HALF_WIDTH
    right_x = center_x + TOP_FRONT_TRIM_HALF_WIDTH
    lower_y = y_at_x(points[15], points[16], left_x)

    # Construct a tangent 1 mm fillet between the incoming lower shoulder and
    # the new vertical trim. The right fillet is its exact mirror.
    incoming_x = left_x - points[15][0]
    incoming_y = lower_y - points[15][1]
    incoming_length = math.hypot(incoming_x, incoming_y)
    unit_x = incoming_x / incoming_length
    unit_y = incoming_y / incoming_length
    turn = math.atan2(unit_x, unit_y)  # clockwise turn into the upward wall
    tangent_distance = TOP_FRONT_TRIM_FILLET_RADIUS / math.tan(abs(turn) / 2.0)
    left_slope_tangent = [
        left_x - unit_x * tangent_distance,
        lower_y - unit_y * tangent_distance,
        0.0,
        0.0,
        math.tan(-abs(turn) / 4.0),
    ]
    left_vertical_tangent = [
        left_x,
        lower_y + tangent_distance,
        0.0,
        0.0,
        0.0,
    ]
    top_y = points[21][1]
    quarter_circle_bulge = -math.tan(math.pi / 8.0)
    left_top_vertical = [
        left_x,
        top_y - TOP_FRONT_UPPER_FILLET_RADIUS,
        0.0,
        0.0,
        quarter_circle_bulge,
    ]
    left_top_horizontal = [
        left_x + TOP_FRONT_UPPER_FILLET_RADIUS,
        top_y,
        0.0,
        0.0,
        0.0,
    ]
    right_top_horizontal = [
        right_x - TOP_FRONT_UPPER_FILLET_RADIUS,
        top_y,
        0.0,
        0.0,
        quarter_circle_bulge,
    ]
    right_top_vertical = [
        right_x,
        top_y - TOP_FRONT_UPPER_FILLET_RADIUS,
        0.0,
        0.0,
        0.0,
    ]
    right_vertical_tangent = [
        right_x,
        left_vertical_tangent[1],
        0.0,
        0.0,
        left_slope_tangent[4],
    ]
    right_slope_tangent = [
        2.0 * center_x - left_slope_tangent[0],
        left_slope_tangent[1],
        0.0,
        0.0,
        0.0,
    ]

    # Replace each protruding shoulder with one vertical cut between its two
    # intersections, retaining the neighboring original profile segments.
    top_profile = [list(point) for point in points[21:48]]
    top_profile[-1][4] = 0.0
    trimmed = (
        points[:16]
        + [left_slope_tangent, left_vertical_tangent]
        + [left_top_vertical, left_top_horizontal]
        + top_profile
        + [right_top_horizontal, right_top_vertical]
        + [right_vertical_tangent, right_slope_tangent]
        + points[53:]
    )
    outline.set_points(trimmed, format="xyseb")
    return 1


def build_dxf():
    source = ezdxf.readfile(SOURCE)
    source_msp = source.modelspace()
    allowed = {"LINE", "ARC", "CIRCLE", "LWPOLYLINE"}

    frame_entities = [
        e for e in source_msp if e.dxftype() in allowed and in_frame_window(e)
    ]
    old_center = {
        e.dxf.handle
        for e in frame_entities
        if e.dxf.layer == "Cut"
        and inside_box(e, *XCORE_CENTER, PATTERN_HALF_DIAGONAL)
    }
    tank_center = [
        e
        for e in source_msp
        if e.dxftype() in allowed
        and e.dxf.layer == "Cut"
        and inside_box(e, *TANK_CENTER, PATTERN_HALF_DIAGONAL)
    ]

    output = ezdxf.new("R2013", setup=True)
    output.units = ezdxf.units.MM
    importer = Importer(source, output)
    importer.import_entities(
        [e for e in frame_entities if e.dxf.handle not in old_center],
        target_layout=output.modelspace(),
    )
    translated_tank = [e.copy() for e in tank_center]
    dx = XCORE_CENTER[0] - TANK_CENTER[0]
    dy = XCORE_CENTER[1] - TANK_CENTER[1]
    inplace(translated_tank, Matrix44.translate(dx, dy, 0.0))
    importer.import_entities(translated_tank, target_layout=output.modelspace())
    importer.finalize()

    frame_box = bbox.extents(output.modelspace(), fast=True)
    widen_bottom_front_profile(output, XCORE_CENTER[0])
    translate_front_keyholes(output, XCORE_CENTER[0], frame_box.extmax.y)

    # Add a conventional 20 x 20 mm M2 mounting square. Relative to the
    # existing 25.5 mm diamond, this pattern is rotated by 45 degrees.
    cx, cy = XCORE_CENTER
    for x_offset in (-NEW_PATTERN_HALF_SPACING, NEW_PATTERN_HALF_SPACING):
        for y_offset in (-NEW_PATTERN_HALF_SPACING, NEW_PATTERN_HALF_SPACING):
            output.modelspace().add_circle(
                (cx + x_offset, cy + y_offset),
                radius=M2_CUT_RADIUS,
                dxfattribs={"layer": "Cut"},
            )

    out = HERE / f"{STEM}.dxf"
    output.saveas(out)
    return output, len(old_center), len(tank_center)


def build_top_plate_dxf():
    source = ezdxf.readfile(SOURCE)
    allowed = {"LINE", "ARC", "CIRCLE", "LWPOLYLINE"}
    entities = [
        entity
        for entity in source.modelspace()
        if entity.dxftype() in allowed
        and in_window(entity, TOP_PLATE_WINDOW)
        and entity.dxf.layer != "Calque1_pocket"
    ]

    output = ezdxf.new("R2013", setup=True)
    output.units = ezdxf.units.MM
    importer = Importer(source, output)
    importer.import_entities(entities, target_layout=output.modelspace())
    importer.finalize()
    top_box = bbox.extents(output.modelspace(), fast=True)
    widen_top_plate_front_profile(output)
    trim_top_plate_front_sides(output, top_box.center.x)
    translate_front_keyholes(output, top_box.center.x, top_box.extmax.y)
    output.saveas(HERE / f"{TOP_STEM}.dxf")
    return output


def cut_polygon(doc):
    segments = []
    for entity in doc.modelspace():
        if entity.dxf.layer not in {"Cut", "Calque1"}:
            continue
        try:
            vertices = list(make_path(entity).flattening(distance=0.025))
        except (TypeError, ValueError):
            continue
        if len(vertices) > 1:
            segments.append(LineString([(v.x, v.y) for v in vertices]))
    faces = list(polygonize(unary_union(segments)))
    if not faces:
        raise RuntimeError("Cut geometry did not polygonize")
    return max(faces, key=lambda p: p.area)


def build_stl(doc, stem=STEM, thickness=3.0):
    polygon = cut_polygon(doc)
    mesh = trimesh.creation.extrude_polygon(polygon, height=thickness)
    mesh.export(HERE / f"{stem}.stl")
    return mesh


def build_pdf(doc, stem=STEM):
    page_w, page_h = landscape(A4)
    margin = 36.0
    drawing_box = bbox.extents(doc.modelspace(), fast=True)
    width = drawing_box.size.x
    height = drawing_box.size.y
    scale = min((page_w - 2 * margin) / width, (page_h - 2 * margin) / height)
    offset_x = (page_w - width * scale) / 2 - drawing_box.extmin.x * scale
    offset_y = (page_h - height * scale) / 2 - drawing_box.extmin.y * scale

    canvas = Canvas(str(HERE / f"{stem}.pdf"), pagesize=(page_w, page_h))
    canvas.setStrokeColorRGB(0.0, 0.36, 1.0)
    canvas.setLineWidth(0.55)
    for entity in doc.modelspace():
        try:
            vertices = list(make_path(entity).flattening(distance=0.025))
        except (TypeError, ValueError):
            continue
        if len(vertices) < 2:
            continue
        path = canvas.beginPath()
        path.moveTo(vertices[0].x * scale + offset_x, vertices[0].y * scale + offset_y)
        for vertex in vertices[1:]:
            path.lineTo(vertex.x * scale + offset_x, vertex.y * scale + offset_y)
        canvas.drawPath(path)
    canvas.showPage()
    canvas.save()


def main():
    doc, removed, inserted = build_dxf()
    mesh = build_stl(doc)
    build_pdf(doc)
    top_doc = build_top_plate_dxf()
    top_mesh = build_stl(top_doc, stem=TOP_STEM, thickness=2.0)
    build_pdf(top_doc, stem=TOP_STEM)
    print(f"Replaced {removed} X-Core center entities with {inserted} Tank entities")
    print(f"STL: {len(mesh.faces)} faces, watertight={mesh.is_watertight}, extents={mesh.extents}")
    print(
        f"Top plate STL: {len(top_mesh.faces)} faces, "
        f"watertight={top_mesh.is_watertight}, extents={top_mesh.extents}"
    )


if __name__ == "__main__":
    main()

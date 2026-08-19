#!/usr/bin/env python3
"""Build a standalone X-Core bottom plate with the V2 Tank center cutouts."""

from pathlib import Path
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

# Centers measured from the source drawing. The FC pattern is a 25.5 mm square
# rotated 45 degrees, so its horizontal/vertical diagonals are 25.5*sqrt(2).
XCORE_CENTER = (116.457, -169.963)
TANK_CENTER = (115.885, -507.515)
PATTERN_HALF_DIAGONAL = 18.2  # 25.5/sqrt(2), with selection tolerance
FRAME_WINDOW = (55.0, -230.0, 178.0, -110.0)
M2_CUT_RADIUS = 1.0
NEW_PATTERN_HALF_SPACING = 10.0


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


def build_stl(doc):
    polygon = cut_polygon(doc)
    mesh = trimesh.creation.extrude_polygon(polygon, height=3.0)
    mesh.export(HERE / f"{STEM}.stl")
    return mesh


def build_pdf(doc):
    page_w, page_h = landscape(A4)
    margin = 36.0
    drawing_box = bbox.extents(doc.modelspace(), fast=True)
    width = drawing_box.size.x
    height = drawing_box.size.y
    scale = min((page_w - 2 * margin) / width, (page_h - 2 * margin) / height)
    offset_x = (page_w - width * scale) / 2 - drawing_box.extmin.x * scale
    offset_y = (page_h - height * scale) / 2 - drawing_box.extmin.y * scale

    canvas = Canvas(str(HERE / f"{STEM}.pdf"), pagesize=(page_w, page_h))
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
    print(f"Replaced {removed} X-Core center entities with {inserted} Tank entities")
    print(f"STL: {len(mesh.faces)} faces, watertight={mesh.is_watertight}, extents={mesh.extents}")


if __name__ == "__main__":
    main()

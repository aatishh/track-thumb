
from build123d import *
from build123d import WorkplaneList
from build123d.topology import Compound
import math
try:
    from ocp_vscode import show
    _has_ocp = True
except ImportError:
    _has_ocp = False
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse as BFuse, BRepAlgoAPI_Cut as BCut, BRepAlgoAPI_Common as BCommon

def fuse_solids(solid1, solid2):
    """Fuse two solids with fuzzy tolerance. Falls back to Compound if they do not intersect."""
    if solid1 is None: return solid2
    if solid2 is None: return solid1
    from OCP.TopAbs import TopAbs_SOLID
    try:
        fuse_op = BFuse(solid1.wrapped, solid2.wrapped)
        fuse_op.SetFuzzyValue(0.01)
        fuse_op.Build()
        result_shape = fuse_op.Shape()
        if not result_shape.IsNull():
            from OCP.TopAbs import TopAbs_COMPOUND
            from OCP.TopExp import TopExp_Explorer
            from OCP.TopoDS import TopoDS
            if result_shape.ShapeType() == TopAbs_SOLID:
                return Solid(result_shape)
            if result_shape.ShapeType() == TopAbs_COMPOUND:
                from OCP.TopAbs import TopAbs_SOLID as _TS
                _exp = TopExp_Explorer(result_shape, _TS)
                _solids = []
                while _exp.More():
                    _solids.append(Solid(TopoDS.Solid_s(_exp.Current())))
                    _exp.Next()
                if len(_solids) == 1:
                    return _solids[0]
                if len(_solids) > 1:
                    return Compound(_solids)
    except:
        pass
    existing = list(solid1.solids()) if isinstance(solid1, Compound) else [solid1]
    new_s = list(solid2.solids()) if isinstance(solid2, Compound) else [solid2]
    return Compound(existing + new_s)

def cut_solids(shape, tool):
    """Cut tool from shape. When tool extends beyond shape (through-all cuts), BCut returns a
    Compound with the cut body AND the tool remainder. _extract_cut_result discards solids
    outside the original bounding box so stray geometry is not returned."""
    if shape is None: return None
    if tool is None: return shape

    def _extract_cut_result(raw_shape, original_solid):
        from OCP.TopAbs import TopAbs_SOLID
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopoDS import TopoDS
        if raw_shape.IsNull(): return original_solid
        exp = TopExp_Explorer(raw_shape, TopAbs_SOLID)
        solids = []
        while exp.More():
            s = Solid(TopoDS.Solid_s(exp.Current()))
            if not s.wrapped.IsNull() and len(list(s.faces())) > 0:
                solids.append(s)
            exp.Next()
        if not solids: return original_solid
        if len(solids) == 1: return solids[0]
        try:
            obb = original_solid.bounding_box()
            tol = 0.5
            kept = []
            for s in solids:
                try:
                    sbb = s.bounding_box()
                    if (sbb.min.X >= obb.min.X - tol and sbb.max.X <= obb.max.X + tol and
                        sbb.min.Y >= obb.min.Y - tol and sbb.max.Y <= obb.max.Y + tol and
                        sbb.min.Z >= obb.min.Z - tol and sbb.max.Z <= obb.max.Z + tol):
                        kept.append(s)
                except:
                    kept.append(s)
            if len(kept) == 1: return kept[0]
            if len(kept) > 1: return Compound(kept)
        except:
            pass
        return max(solids, key=lambda s: len(list(s.faces())))

    try:
        if isinstance(shape, Compound):
            result_solids = []
            for solid in shape.solids():
                try:
                    sbb = solid.bounding_box()
                    tbb = tool.bounding_box()
                    overlap = (
                        not (sbb.max.X < tbb.min.X or sbb.min.X > tbb.max.X) and
                        not (sbb.max.Y < tbb.min.Y or sbb.min.Y > tbb.max.Y) and
                        not (sbb.max.Z < tbb.min.Z or sbb.min.Z > tbb.max.Z)
                    )
                    if overlap:
                        cut_op = BCut(solid.wrapped, tool.wrapped)
                        cut_op.SetFuzzyValue(0.01)
                        cut_op.Build()
                        result_solids.append(_extract_cut_result(cut_op.Shape(), solid))
                    else:
                        result_solids.append(solid)
                except:
                    result_solids.append(solid)
            return result_solids[0] if len(result_solids) == 1 else Compound(result_solids)
        else:
            sbb = shape.bounding_box()
            tbb = tool.bounding_box()
            overlap = (
                not (sbb.max.X < tbb.min.X or sbb.min.X > tbb.max.X) and
                not (sbb.max.Y < tbb.min.Y or sbb.min.Y > tbb.max.Y) and
                not (sbb.max.Z < tbb.min.Z or sbb.min.Z > tbb.max.Z)
            )
            if overlap:
                cut_op = BCut(shape.wrapped, tool.wrapped)
                cut_op.SetFuzzyValue(0.01)
                cut_op.Build()
                return _extract_cut_result(cut_op.Shape(), shape)
            return shape
    except:
        return shape

def intersect_solids(solid1, solid2):
    """Intersect two solids (keep only overlapping volume). Used to clip a
    mirrored cut tool to its companion body so the subtract stays bounded.
    Returns solid1 unchanged if either input is None or BCommon fails."""
    if solid1 is None or solid2 is None:
        return solid1
    try:
        common_op = BCommon(solid1.wrapped, solid2.wrapped)
        common_op.SetFuzzyValue(0.01)
        common_op.Build()
        result = common_op.Shape()
        if result is None or result.IsNull():
            return solid1
        from OCP.TopAbs import TopAbs_SOLID
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopoDS import TopoDS
        exp = TopExp_Explorer(result, TopAbs_SOLID)
        solids = []
        while exp.More():
            solids.append(Solid(TopoDS.Solid_s(exp.Current())))
            exp.Next()
        if len(solids) == 1:
            return solids[0]
        if len(solids) > 1:
            return Compound(solids)
        return solid1
    except Exception:
        return solid1
_pl_1 = Plane(
    origin=Vector(-18.5737, -18.4741, 47.5662),
    x_dir=Vector(0.705203, -0.709006, -0.0),
    z_dir=Vector(-0.342038, -0.340204, 0.87594),
)
with BuildSketch(_pl_1) as sk_Sketch1:
    with BuildLine():
        RadiusArc((-64.1402, -19.9119), (-66.2179, -16.2123), -2.999)
        Line((-66.2179, -16.2123), (-78.7322, -12.6921))
        RadiusArc((-78.7322, -12.6921), (-82.4311, -14.7686), -2.9985)
        Line((-82.4311, -14.7686), (-84.3188, -21.4809))
        RadiusArc((-84.3188, -21.4809), (-85.9856, -25.3297), 15.7441)
        Line((-85.9856, -25.3297), (-87.3033, -32.0976))
        Line((-87.3033, -32.0976), (-69.0138, -37.2398))
        Line((-69.0138, -37.2398), (-64.1402, -19.9119))
    _edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch1 = Wire.combine(_edges_sk_Sketch1)[0]
_w_sk_Sketch1 = _w_sk_Sketch1.moved(_pl_1.location)
_mf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_pl_1.wrapped, _w_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mf_sk_Sketch1.Face())
_surf_2 = Plane(
    origin=Vector(-18.5736, -18.4741, 47.5663),
    x_dir=Vector(0.705203, -0.709006, -0.0),
    z_dir=Vector(-0.342038, -0.340204, 0.87594),
)
with BuildSketch(_surf_2) as sk_Sketch2_2:
    with BuildLine():
        Line((-84.1594, -30.3314), (-80.3966, -16.9498))
        Line((-80.3966, -16.9498), (-67.0163, -20.7141))
        Line((-67.0163, -20.7141), (-70.7791, -34.0958))
        Line((-70.7791, -34.0958), (-84.1594, -30.3314))
    _edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch2_2 = Wire.combine(_edges_sk_Sketch2_2)[0]
_w_sk_Sketch2_2 = _w_sk_Sketch2_2.moved(_surf_2.location)
_mf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_surf_2.wrapped, _w_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mf_sk_Sketch2_2.Face())
_ref_plane_3 = Plane(
    origin=Vector(-24.7305, -20.3356, 51.0392),
    x_dir=Vector(0.635133, -0.772403, 0.0),
    z_dir=Vector(-0.410461, -0.337515, 0.847116),
)
with BuildSketch(_ref_plane_3) as sk_Sketch4_3:
    with BuildLine():
        Line((-79.4631, -57.6953), (-76.4801, -57.9079))
        Line((-76.4801, -57.9079), (-73.7344, -58.0849))
        Line((-73.7344, -58.0849), (-71.316, -58.0828))
        Line((-71.316, -58.0828), (-68.4008, -58.2344))
        Line((-68.4008, -58.2344), (-65.4864, -58.3583))
        Line((-65.4864, -58.3583), (-64.0447, -58.4036))
        Line((-64.0447, -58.4036), (-63.8723, -56.8443))
        Line((-63.8723, -56.8443), (-63.7443, -55.2806))
        Line((-63.7443, -55.2806), (-63.5575, -52.1564))
        Line((-63.5575, -52.1564), (-63.275, -50.3687))
        RadiusArc((-63.275, -50.3687), (-63.0784, -47.8317), -16.0482)
        Line((-63.0784, -47.8317), (-62.9064, -47.2049))
        Line((-62.9064, -47.2049), (-62.4787, -45.6394))
        Line((-62.4787, -45.6394), (-62.0731, -44.064))
        Line((-62.0731, -44.064), (-61.6913, -42.4592))
        RadiusArc((-61.6913, -42.4592), (-62.649, -41.3915), -0.8725)
        Line((-62.649, -41.3915), (-67.3462, -41.1347))
        Line((-67.3462, -41.1347), (-72.0436, -40.8756))
        Line((-72.0436, -40.8756), (-76.7423, -40.6178))
        Line((-76.7423, -40.6178), (-81.4397, -40.3586))
        Line((-81.4397, -40.3586), (-82.4315, -40.3055))
        Line((-82.4315, -40.3055), (-82.5385, -42.244))
        Line((-82.5385, -42.244), (-82.6453, -44.1881))
        Line((-82.6453, -44.1881), (-82.8572, -48.0722))
        Line((-82.8572, -48.0722), (-82.9656, -50.0136))
        Line((-82.9656, -50.0136), (-83.0723, -51.9551))
        Line((-83.0723, -51.9551), (-83.1791, -53.8976))
        Line((-83.1791, -53.8976), (-83.2874, -55.838))
        RadiusArc((-83.2874, -55.838), (-83.15, -56.9687), -2.9692)
        Line((-83.15, -56.9687), (-82.9174, -57.4604))
        Line((-82.9174, -57.4604), (-81.775, -57.5058))
        Line((-81.775, -57.5058), (-79.4631, -57.6953))
    _edges_sk_Sketch4_3 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch4_3 = Wire.combine(_edges_sk_Sketch4_3)[0]
_w_sk_Sketch4_3 = _w_sk_Sketch4_3.moved(_ref_plane_3.location)
_mf_sk_Sketch4_3 = BRepBuilderAPI_MakeFace(_ref_plane_3.wrapped, _w_sk_Sketch4_3.wrapped, True)
_face_sk_Sketch4_3 = Face(_mf_sk_Sketch4_3.Face())
_work_plane_4 = Plane(
    origin=Vector(-24.6578, -20.3986, 50.9343),
    x_dir=Vector(0.637423, -0.770514, 0.0),
    z_dir=Vector(-0.409915, -0.339111, 0.846743),
)
with BuildSketch(_work_plane_4) as sk_Sketch7_4:
    with BuildLine():
        Line((-66.2638, -42.0478), (-80.1412, -41.2499))
        Line((-80.1412, -41.2499), (-80.9397, -55.1257))
        Line((-80.9397, -55.1257), (-67.0636, -55.926))
        Line((-67.0636, -55.926), (-66.2638, -42.0478))
    _edges_sk_Sketch7_4 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch7_4 = Wire.combine(_edges_sk_Sketch7_4)[0]
_w_sk_Sketch7_4 = _w_sk_Sketch7_4.moved(_work_plane_4.location)
_mf_sk_Sketch7_4 = BRepBuilderAPI_MakeFace(_work_plane_4.wrapped, _w_sk_Sketch7_4.wrapped, True)
_face_sk_Sketch7_4 = Face(_mf_sk_Sketch7_4.Face())
_sketch_plane_5 = Plane(
    origin=Vector(-37.1881, 61.8636, 9.4875),
    x_dir=Vector(0.857065, 0.515209, 0.0),
    z_dir=Vector(0.510815, -0.849755, -0.13032),
)
with BuildSketch(_sketch_plane_5) as sk_Sketch9_5:
    with BuildLine():
        Line((-65.4419, 21.9484), (-61.0233, 24.2921))
        Line((-61.0233, 24.2921), (-62.4235, 26.915))
        Line((-62.4235, 26.915), (-66.8348, 24.576))
        Line((-66.8348, 24.576), (-65.4419, 21.9484))
    _edges_sk_Sketch9_5 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch9_5 = Wire.combine(_edges_sk_Sketch9_5)[0]
_w_sk_Sketch9_5 = _w_sk_Sketch9_5.moved(_sketch_plane_5.location)
_mf_sk_Sketch9_5 = BRepBuilderAPI_MakeFace(_sketch_plane_5.wrapped, _w_sk_Sketch9_5.wrapped, True)
_face_sk_Sketch9_5 = Face(_mf_sk_Sketch9_5.Face())
_wp_6 = Plane(
    origin=Vector(0.0, 0.0, 6.9074),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_wp_6) as sk_Sketch3_6:
    with BuildLine():
        RadiusArc((-99.0746, -31.8174), (-99.4868, -30.6163), -10.7667)
        Line((-99.4868, -30.6163), (-100.1209, -29.452))
        RadiusArc((-100.1209, -29.452), (-102.0174, -27.4405), -8.9899)
        Line((-102.0174, -27.4405), (-111.2759, -20.2483))
        RadiusArc((-111.2759, -20.2483), (-113.2453, -18.4637), 16.3751)
        RadiusArc((-113.2453, -18.4637), (-114.864, -16.4358), 18.4778)
        RadiusArc((-114.864, -16.4358), (-115.9734, -14.3395), 10.189)
        RadiusArc((-115.9734, -14.3395), (-116.4627, -12.0422), 7.3612)
        Line((-116.4627, -12.0422), (-116.3807, -11.0344))
        Line((-116.3807, -11.0344), (-116.8882, -10.9854))
        RadiusArc((-116.8882, -10.9854), (-115.6962, -8.6294), 6.926)
        Line((-115.6962, -8.6294), (-107.0162, 2.5506))
        RadiusArc((-107.0162, 2.5506), (-106.0021, 3.5126), 3.1426)
        RadiusArc((-106.0021, 3.5126), (-104.2802, 3.9386), 3.5478)
        RadiusArc((-104.2802, 3.9386), (-103.0622, 3.6466), 2.8054)
        Line((-103.0622, 3.6466), (-101.8162, 3.0286))
        RadiusArc((-101.8162, 3.0286), (-99.9502, 1.7606), 47.0253)
        Line((-99.9502, 1.7606), (-88.2682, -6.7854))
        Line((-88.2682, -6.7854), (-83.2562, -10.6854))
        Line((-83.2562, -10.6854), (-79.3394, -11.8984))
        Line((-79.3394, -11.8984), (-75.8157, -12.5676))
        Line((-75.8157, -12.5676), (-72.2854, -13.811))
        RadiusArc((-72.2854, -13.811), (-67.9087, -14.932), -43.896)
        Line((-67.9087, -14.932), (-55.0264, -17.5334))
        RadiusArc((-55.0264, -17.5334), (-53.5552, -18.1979), 2.9976)
        RadiusArc((-53.5552, -18.1979), (-52.9913, -19.0821), 2.3549)
        RadiusArc((-52.9913, -19.0821), (-52.8812, -19.6165), 1.9525)
        Line((-52.8812, -19.6165), (-52.8671, -20.1561))
        Line((-52.8671, -20.1561), (-52.0, -19.0))
        Line((-52.0, -19.0), (-49.0221, -16.6177))
        RadiusArc((-49.0221, -16.6177), (-50.486, -15.0761), -4.5978)
        RadiusArc((-50.486, -15.0761), (-53.0129, -13.999), -6.4565)
        Line((-53.0129, -13.999), (-73.7618, -10.0384))
        Line((-73.7618, -10.0384), (-76.6877, -7.3929))
        Line((-76.6877, -7.3929), (-76.0, -7.0))
        Line((-76.0, -7.0), (-72.8937, -5.8352))
        RadiusArc((-72.8937, -5.8352), (-70.9722, -4.6414), 10.1203)
        RadiusArc((-70.9722, -4.6414), (-69.0802, -4.0514), 8.52)
        RadiusArc((-69.0802, -4.0514), (-65.0462, 1.5406), -5.0036)
        RadiusArc((-65.0462, 1.5406), (-66.5302, 4.6246), -5.741)
        RadiusArc((-66.5302, 4.6246), (-68.6382, 7.4206), -69.3315)
        RadiusArc((-68.6382, 7.4206), (-71.5402, 10.1466), -13.7405)
        RadiusArc((-71.5402, 10.1466), (-73.5802, 11.3206), -24.6603)
        RadiusArc((-73.5802, 11.3206), (-75.1062, 11.5026), -2.2061)
        RadiusArc((-75.1062, 11.5026), (-76.1242, 10.8146), -1.7568)
        Line((-76.1242, 10.8146), (-78.1042, 8.0366))
        Line((-78.1042, 8.0366), (-80.1302, 4.7606))
        RadiusArc((-80.1302, 4.7606), (-82.3362, 1.1646), 21.0071)
        RadiusArc((-82.3362, 1.1646), (-86.1562, -1.1074), 5.9408)
        RadiusArc((-86.1562, -1.1074), (-91.8282, -0.2054), 11.4991)
        RadiusArc((-91.8282, -0.2054), (-95.3622, 1.6146), 13.6049)
        RadiusArc((-95.3622, 1.6146), (-99.5202, 4.1146), -30.0367)
        Line((-99.5202, 4.1146), (-100.8281, 4.7226))
        RadiusArc((-100.8281, 4.7226), (-103.4762, 5.5706), -11.9544)
        RadiusArc((-103.4762, 5.5706), (-106.3561, 5.4606), -6.2798)
        RadiusArc((-106.3561, 5.4606), (-108.5942, 3.7786), -5.321)
        Line((-108.5942, 3.7786), (-117.2742, -7.4034))
        RadiusArc((-117.2742, -7.4034), (-118.2112, -8.8672), -9.9735)
        Line((-118.2112, -8.8672), (-118.3529, -10.6108))
        RadiusArc((-118.3529, -10.6108), (-118.4668, -12.012), -8.5355)
        RadiusArc((-118.4668, -12.012), (-116.5329, -17.5398), -11.8428)
        RadiusArc((-116.5329, -17.5398), (-112.5316, -21.8054), -18.7123)
        Line((-112.5316, -21.8054), (-103.3017, -28.9754))
        RadiusArc((-103.3017, -28.9754), (-101.0144, -32.3264), 5.896)
        RadiusArc((-101.0144, -32.3264), (-100.7852, -35.3791), 9.0444)
        Line((-100.7852, -35.3791), (-101.4334, -42.1733))
        Line((-101.4334, -42.1733), (-97.2818, -40.423))
        Line((-97.2818, -40.423), (-93.5024, -41.5728))
        RadiusArc((-93.5024, -41.5728), (-89.7363, -38.8135), -4.8315)
        RadiusArc((-89.7363, -38.8135), (-89.5476, -34.8797), -5.2266)
        RadiusArc((-89.5476, -34.8797), (-92.2655, -32.0296), -5.0214)
        RadiusArc((-92.2655, -32.0296), (-95.4372, -31.7737), -4.9794)
        RadiusArc((-95.4372, -31.7737), (-98.1211, -33.4831), -5.0208)
        Line((-98.1211, -33.4831), (-98.8126, -33.5499))
        Line((-98.8126, -33.5499), (-99.0746, -31.8174))
    _edges_sk_Sketch3_6 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch3_6 = Wire.combine(_edges_sk_Sketch3_6)[0]
_w_sk_Sketch3_6 = _w_sk_Sketch3_6.moved(_wp_6.location)
_mf_sk_Sketch3_6 = BRepBuilderAPI_MakeFace(_wp_6.wrapped, _w_sk_Sketch3_6.wrapped, True)
_face_sk_Sketch3_6 = Face(_mf_sk_Sketch3_6.Face())
_pl_7 = Plane(
    origin=Vector(0.0, 0.0, 10.9074),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_pl_7) as sk_Sketch10_7:
    with Locations((70.0021, 0.8626)):
        Circle(radius=2.95)
_surf_8 = Plane(
    origin=Vector(0.0, 0.0, 10.9074),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_surf_8) as sk_Sketch11_8:
    with Locations((94.2326, -36.6264)):
        Circle(radius=3.0)
_ref_plane_9 = Plane(
    origin=Vector(0.0, 0.0, 10.9074),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_ref_plane_9) as sk_Sketch13_9:
    with BuildLine():
        Line((92.1222, -4.0094), (85.9002, -4.9114))
        Line((85.9002, -4.9114), (80.3482, -8.8854))
        RadiusArc((80.3482, -8.8854), (79.3021, -8.8639), 0.6751)
        Line((79.3021, -8.8639), (73.7618, -10.0384))
        Line((73.7618, -10.0384), (53.0129, -13.999))
        RadiusArc((53.0129, -13.999), (50.486, -15.0761), -6.4565)
        RadiusArc((50.486, -15.0761), (49.0221, -16.6177), -4.5978)
        Line((49.0221, -16.6177), (52.0, -19.0))
        Line((52.0, -19.0), (52.8671, -20.1561))
        Line((52.8671, -20.1561), (52.8632, -19.902))
        Line((52.8632, -19.902), (52.8812, -19.6165))
        RadiusArc((52.8812, -19.6165), (52.9913, -19.0821), 1.9525)
        RadiusArc((52.9913, -19.0821), (53.5552, -18.1979), 2.3549)
        RadiusArc((53.5552, -18.1979), (55.0264, -17.5334), 2.9976)
        Line((55.0264, -17.5334), (67.9087, -14.932))
        RadiusArc((67.9087, -14.932), (72.2854, -13.811), -43.896)
        Line((72.2854, -13.811), (74.6506, -13.0102))
        Line((74.6506, -13.0102), (75.8157, -12.5676))
        Line((75.8157, -12.5676), (79.3394, -11.8984))
        Line((79.3394, -11.8984), (83.2562, -10.6854))
        Line((83.2562, -10.6854), (84.9142, -9.3654))
        Line((84.9142, -9.3654), (86.5802, -8.0574))
        Line((86.5802, -8.0574), (88.2682, -6.7854))
        Line((88.2682, -6.7854), (92.1222, -4.0094))
    _edges_sk_Sketch13_9 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch13_9 = Wire.combine(_edges_sk_Sketch13_9)[0]
_w_sk_Sketch13_9 = _w_sk_Sketch13_9.moved(_ref_plane_9.location)
_mf_sk_Sketch13_9 = BRepBuilderAPI_MakeFace(_ref_plane_9.wrapped, _w_sk_Sketch13_9.wrapped, True)
_face_sk_Sketch13_9 = Face(_mf_sk_Sketch13_9.Face())
_pl_10 = Plane(
    origin=Vector(-27.5801, -0.015, 47.7708),
    x_dir=Vector(-0.000545, 1.0, 0.0),
    z_dir=Vector(0.499995, 0.000273, -0.866028),
)
with BuildSketch(_pl_10) as sk_Sketch12_10:
    with BuildLine():
        RadiusArc((16.7728, -1.0268), (12.7527, -0.7958), -198.1264)
        RadiusArc((12.7527, -0.7958), (5.8972, -1.8196), -28.2219)
        RadiusArc((5.8972, -1.8196), (-3.8123, -7.0704), -26.6509)
        RadiusArc((-3.8123, -7.0704), (-10.4682, -15.8754), -25.3827)
        RadiusArc((-10.4682, -15.8754), (-12.8398, -29.2141), -27.1529)
        RadiusArc((-12.8398, -29.2141), (-11.7715, -38.5475), -124.29)
        RadiusArc((-11.7715, -38.5475), (-11.4529, -43.8769), 29.0316)
        Line((-11.4529, -43.8769), (-11.5179, -46.1448))
        Line((-11.5179, -46.1448), (-8.9993, -47.3704))
        Line((-8.9993, -47.3704), (-3.9661, -49.8205))
        Line((-3.9661, -49.8205), (1.067, -52.2705))
        Line((1.067, -52.2705), (6.1002, -54.7233))
        Line((6.1002, -54.7233), (9.725, -56.4886))
        Line((9.725, -56.4886), (10.8386, -55.6771))
        RadiusArc((10.8386, -55.6771), (11.9943, -54.9634), 8.8916)
        Line((11.9943, -54.9634), (13.192, -54.3275))
        RadiusArc((13.192, -54.3275), (15.7115, -53.2358), 27.5457)
        RadiusArc((15.7115, -53.2358), (22.3905, -51.0876), 73.3614)
        RadiusArc((22.3905, -51.0876), (30.4082, -46.2201), -29.2828)
        RadiusArc((30.4082, -46.2201), (36.1747, -38.8251), -25.4217)
        Line((36.1747, -38.8251), (37.2975, -36.3783))
        Line((37.2975, -36.3783), (37.2705, -34.0988))
        Line((37.2705, -34.0988), (37.0603, -29.5497))
        Line((37.0603, -29.5497), (36.9152, -27.2775))
        Line((36.9152, -27.2775), (36.5931, -22.7329))
        Line((36.5931, -22.7329), (36.249, -18.1912))
        RadiusArc((36.249, -18.1912), (36.8262, -16.5251), 2.2161)
        Line((36.8262, -16.5251), (36.7239, -15.6976))
        Line((36.7239, -15.6976), (33.6781, -11.1654))
        Line((33.6781, -11.1654), (31.3142, -8.1632))
        Line((31.3142, -8.1632), (28.4363, -4.0679))
        Line((28.4363, -4.0679), (24.5622, -0.7727))
        Line((24.5622, -0.7727), (24.2924, -0.713))
        Line((24.2924, -0.713), (24.0542, -0.6602))
        RadiusArc((24.0542, -0.6602), (20.043, -1.3254), 8.599)
        RadiusArc((20.043, -1.3254), (16.7728, -1.0268), 98.1605)
    _edges_sk_Sketch12_10 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch12_10 = Wire.combine(_edges_sk_Sketch12_10)[0]
_w_sk_Sketch12_10 = _w_sk_Sketch12_10.moved(_pl_10.location)
_mf_sk_Sketch12_10 = BRepBuilderAPI_MakeFace(_pl_10.wrapped, _w_sk_Sketch12_10.wrapped, True)
_face_sk_Sketch12_10 = Face(_mf_sk_Sketch12_10.Face())
_pl_11 = Plane(
    origin=Vector(-24.9903, 4.5437, 50.8),
    x_dir=Vector(0.178885, 0.98387, 0.0),
    z_dir=Vector(0.44, -0.08, -0.894427),
)
with BuildSketch(_pl_11) as sk_Sketch17_11:
    with BuildLine():
        Line((25.4523, -24.1495), (28.6385, -24.1495))
        Line((28.6385, -24.1495), (30.1721, -24.1495))
        Line((30.1721, -24.1495), (23.6603, -7.5362))
        Line((23.6603, -7.5362), (19.8517, -8.4971))
        Line((19.8517, -8.4971), (17.2478, -8.4971))
        Line((17.2478, -8.4971), (17.474, -13.1096))
        RadiusArc((17.474, -13.1096), (25.4523, -24.1495), 22.7906)
    _edges_sk_Sketch17_11 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch17_11 = Wire.combine(_edges_sk_Sketch17_11)[0]
_w_sk_Sketch17_11 = _w_sk_Sketch17_11.moved(_pl_11.location)
_mf_sk_Sketch17_11 = BRepBuilderAPI_MakeFace(_pl_11.wrapped, _w_sk_Sketch17_11.wrapped, True)
_face_sk_Sketch17_11 = Face(_mf_sk_Sketch17_11.Face())
sk_Sketch16_loft_12_edges = [
    Edge.make_line((101.4334, -42.1733, 0), (101.4063, -41.8905, 0)),
    Edge.make_line((101.4063, -41.8905, 0), (101.3792, -41.6077, 0)),
    Edge.make_line((101.3792, -41.6077, 0), (101.352, -41.3249, 0)),
    Edge.make_line((101.352, -41.3249, 0), (101.3249, -41.042, 0)),
    Edge.make_line((101.3249, -41.042, 0), (101.2978, -40.7592, 0)),
    Edge.make_line((101.2978, -40.7592, 0), (101.2707, -40.4764, 0)),
    Edge.make_line((101.2707, -40.4764, 0), (101.2436, -40.1936, 0)),
    Edge.make_line((101.2436, -40.1936, 0), (101.2164, -39.9108, 0)),
    Edge.make_line((101.2164, -39.9108, 0), (101.1893, -39.628, 0)),
    Edge.make_line((101.1893, -39.628, 0), (101.1622, -39.3452, 0)),
    Edge.make_line((101.1622, -39.3452, 0), (101.1351, -39.0624, 0)),
    Edge.make_line((101.1351, -39.0624, 0), (101.1079, -38.7795, 0)),
    Edge.make_line((101.1079, -38.7795, 0), (101.0808, -38.4967, 0)),
    Edge.make_line((101.0808, -38.4967, 0), (101.0537, -38.2139, 0)),
    Edge.make_line((101.0537, -38.2139, 0), (101.0266, -37.9311, 0)),
    Edge.make_line((101.0266, -37.9311, 0), (100.9995, -37.6483, 0)),
    Edge.make_line((100.9995, -37.6483, 0), (100.9723, -37.3655, 0)),
    Edge.make_line((100.9723, -37.3655, 0), (100.9452, -37.0827, 0)),
    Edge.make_line((100.9452, -37.0827, 0), (100.9181, -36.7999, 0)),
    Edge.make_line((100.9181, -36.7999, 0), (100.891, -36.517, 0)),
    Edge.make_line((100.891, -36.517, 0), (100.8639, -36.2342, 0)),
    Edge.make_line((100.8639, -36.2342, 0), (100.8367, -35.9514, 0)),
    Edge.make_line((100.8367, -35.9514, 0), (100.7833, -35.6976, 0)),
    Edge.make_line((100.7833, -35.6976, 0), (100.4992, -35.6976, 0)),
    Edge.make_line((100.4992, -35.6976, 0), (100.215, -35.6976, 0)),
    Edge.make_line((100.215, -35.6976, 0), (99.9309, -35.6976, 0)),
    Edge.make_line((99.9309, -35.6976, 0), (99.6468, -35.6976, 0)),
    Edge.make_line((99.6468, -35.6976, 0), (99.3627, -35.6976, 0)),
    Edge.make_line((99.3627, -35.6976, 0), (99.0786, -35.6976, 0)),
    Edge.make_line((99.0786, -35.6976, 0), (98.7967, -35.7032, 0)),
    Edge.make_line((98.7967, -35.7032, 0), (98.5859, -35.8938, 0)),
    Edge.make_line((98.5859, -35.8938, 0), (98.3752, -36.0843, 0)),
    Edge.make_line((98.3752, -36.0843, 0), (98.1645, -36.2749, 0)),
    Edge.make_line((98.1645, -36.2749, 0), (97.9537, -36.4654, 0)),
    Edge.make_line((97.9537, -36.4654, 0), (97.7839, -36.6766, 0)),
    Edge.make_line((97.7839, -36.6766, 0), (97.7567, -36.9594, 0)),
    Edge.make_line((97.7567, -36.9594, 0), (97.7294, -37.2422, 0)),
    Edge.make_line((97.7294, -37.2422, 0), (97.7022, -37.525, 0)),
    Edge.make_line((97.7022, -37.525, 0), (97.6749, -37.8078, 0)),
    Edge.make_line((97.6749, -37.8078, 0), (97.6476, -38.0906, 0)),
    Edge.make_line((97.6476, -38.0906, 0), (97.6204, -38.3734, 0)),
    Edge.make_line((97.6204, -38.3734, 0), (97.5931, -38.6562, 0)),
    Edge.make_line((97.5931, -38.6562, 0), (97.5659, -38.939, 0)),
    Edge.make_line((97.5659, -38.939, 0), (97.5386, -39.2218, 0)),
    Edge.make_line((97.5386, -39.2218, 0), (97.5114, -39.5046, 0)),
    Edge.make_line((97.5114, -39.5046, 0), (97.4841, -39.7874, 0)),
    Edge.make_line((97.4841, -39.7874, 0), (97.4569, -40.0702, 0)),
    Edge.make_line((97.4569, -40.0702, 0), (97.4296, -40.353, 0)),
    Edge.make_line((97.4296, -40.353, 0), (97.4765, -40.5906, 0)),
    Edge.make_line((97.4765, -40.5906, 0), (97.7403, -40.6961, 0)),
    Edge.make_line((97.7403, -40.6961, 0), (98.0041, -40.8016, 0)),
    Edge.make_line((98.0041, -40.8016, 0), (98.2679, -40.9071, 0)),
    Edge.make_line((98.2679, -40.9071, 0), (98.5317, -41.0126, 0)),
    Edge.make_line((98.5317, -41.0126, 0), (98.7955, -41.1182, 0)),
    Edge.make_line((98.7955, -41.1182, 0), (99.0593, -41.2237, 0)),
    Edge.make_line((99.0593, -41.2237, 0), (99.3231, -41.3292, 0)),
    Edge.make_line((99.3231, -41.3292, 0), (99.5869, -41.4347, 0)),
    Edge.make_line((99.5869, -41.4347, 0), (99.8507, -41.5402, 0)),
    Edge.make_line((99.8507, -41.5402, 0), (100.1144, -41.6457, 0)),
    Edge.make_line((100.1144, -41.6457, 0), (100.3782, -41.7512, 0)),
    Edge.make_line((100.3782, -41.7512, 0), (100.642, -41.8568, 0)),
    Edge.make_line((100.642, -41.8568, 0), (100.9058, -41.9623, 0)),
    Edge.make_line((100.9058, -41.9623, 0), (101.1696, -42.0678, 0)),
    Edge.make_line((101.1696, -42.0678, 0), (101.4334, -42.1733, 0)),
]
sk_Sketch16_loft_12_wire = Wire(sk_Sketch16_loft_12_edges)
sk_Sketch16_loft_12_face = Face(sk_Sketch16_loft_12_wire)
sk_Sketch16_loft_12_face = Plane(origin=Vector(0.0, 0.0, 10.9074), x_dir=Vector(-1.0, 0.0, 0.0), z_dir=Vector(0.0, 0.0, 1.0)) * sk_Sketch16_loft_12_face
sk_Sketch18_loft_13_edges = [
    Edge.make_line((101.4334, -42.1733, 0), (101.4047, -41.8735, 0)),
    Edge.make_line((101.4047, -41.8735, 0), (101.3759, -41.5737, 0)),
    Edge.make_line((101.3759, -41.5737, 0), (101.3472, -41.2739, 0)),
    Edge.make_line((101.3472, -41.2739, 0), (101.3184, -40.9741, 0)),
    Edge.make_line((101.3184, -40.9741, 0), (101.2897, -40.6743, 0)),
    Edge.make_line((101.2897, -40.6743, 0), (101.2609, -40.3745, 0)),
    Edge.make_line((101.2609, -40.3745, 0), (101.2322, -40.0747, 0)),
    Edge.make_line((101.2322, -40.0747, 0), (101.2034, -39.7749, 0)),
    Edge.make_line((101.2034, -39.7749, 0), (101.1747, -39.4751, 0)),
    Edge.make_line((101.1747, -39.4751, 0), (101.1459, -39.1753, 0)),
    Edge.make_line((101.1459, -39.1753, 0), (101.1172, -38.8755, 0)),
    Edge.make_line((101.1172, -38.8755, 0), (101.0884, -38.5757, 0)),
    Edge.make_line((101.0884, -38.5757, 0), (101.0597, -38.2759, 0)),
    Edge.make_line((101.0597, -38.2759, 0), (101.0309, -37.9762, 0)),
    Edge.make_line((101.0309, -37.9762, 0), (101.0022, -37.6764, 0)),
    Edge.make_line((101.0022, -37.6764, 0), (100.9734, -37.3766, 0)),
    Edge.make_line((100.9734, -37.3766, 0), (100.9447, -37.0768, 0)),
    Edge.make_line((100.9447, -37.0768, 0), (100.9159, -36.777, 0)),
    Edge.make_line((100.9159, -36.777, 0), (100.8872, -36.4772, 0)),
    Edge.make_line((100.8872, -36.4772, 0), (100.8584, -36.1774, 0)),
    Edge.make_line((100.8584, -36.1774, 0), (100.8297, -35.8776, 0)),
    Edge.make_line((100.8297, -35.8776, 0), (100.6993, -35.6563, 0)),
    Edge.make_line((100.6993, -35.6563, 0), (100.4164, -35.553, 0)),
    Edge.make_line((100.4164, -35.553, 0), (100.1336, -35.4496, 0)),
    Edge.make_line((100.1336, -35.4496, 0), (99.8507, -35.3463, 0)),
    Edge.make_line((99.8507, -35.3463, 0), (99.5678, -35.243, 0)),
    Edge.make_line((99.5678, -35.243, 0), (99.2849, -35.1397, 0)),
    Edge.make_line((99.2849, -35.1397, 0), (99.002, -35.0363, 0)),
    Edge.make_line((99.002, -35.0363, 0), (98.7313, -34.9956, 0)),
    Edge.make_line((98.7313, -34.9956, 0), (98.5079, -35.1976, 0)),
    Edge.make_line((98.5079, -35.1976, 0), (98.2845, -35.3996, 0)),
    Edge.make_line((98.2845, -35.3996, 0), (98.0611, -35.6016, 0)),
    Edge.make_line((98.0611, -35.6016, 0), (97.8377, -35.8036, 0)),
    Edge.make_line((97.8377, -35.8036, 0), (97.6143, -36.0056, 0)),
    Edge.make_line((97.6143, -36.0056, 0), (97.4082, -36.2154, 0)),
    Edge.make_line((97.4082, -36.2154, 0), (97.403, -36.5165, 0)),
    Edge.make_line((97.403, -36.5165, 0), (97.3979, -36.8177, 0)),
    Edge.make_line((97.3979, -36.8177, 0), (97.3927, -37.1188, 0)),
    Edge.make_line((97.3927, -37.1188, 0), (97.3875, -37.4199, 0)),
    Edge.make_line((97.3875, -37.4199, 0), (97.3824, -37.721, 0)),
    Edge.make_line((97.3824, -37.721, 0), (97.3772, -38.0222, 0)),
    Edge.make_line((97.3772, -38.0222, 0), (97.372, -38.3233, 0)),
    Edge.make_line((97.372, -38.3233, 0), (97.3669, -38.6244, 0)),
    Edge.make_line((97.3669, -38.6244, 0), (97.3617, -38.9255, 0)),
    Edge.make_line((97.3617, -38.9255, 0), (97.3565, -39.2267, 0)),
    Edge.make_line((97.3565, -39.2267, 0), (97.3514, -39.5278, 0)),
    Edge.make_line((97.3514, -39.5278, 0), (97.3462, -39.8289, 0)),
    Edge.make_line((97.3462, -39.8289, 0), (97.341, -40.1301, 0)),
    Edge.make_line((97.341, -40.1301, 0), (97.3359, -40.4312, 0)),
    Edge.make_line((97.3359, -40.4312, 0), (97.5185, -40.6075, 0)),
    Edge.make_line((97.5185, -40.6075, 0), (97.7982, -40.7193, 0)),
    Edge.make_line((97.7982, -40.7193, 0), (98.0778, -40.8312, 0)),
    Edge.make_line((98.0778, -40.8312, 0), (98.3574, -40.943, 0)),
    Edge.make_line((98.3574, -40.943, 0), (98.6371, -41.0548, 0)),
    Edge.make_line((98.6371, -41.0548, 0), (98.9167, -41.1667, 0)),
    Edge.make_line((98.9167, -41.1667, 0), (99.1963, -41.2785, 0)),
    Edge.make_line((99.1963, -41.2785, 0), (99.476, -41.3904, 0)),
    Edge.make_line((99.476, -41.3904, 0), (99.7556, -41.5022, 0)),
    Edge.make_line((99.7556, -41.5022, 0), (100.0352, -41.6141, 0)),
    Edge.make_line((100.0352, -41.6141, 0), (100.3149, -41.7259, 0)),
    Edge.make_line((100.3149, -41.7259, 0), (100.5945, -41.8378, 0)),
    Edge.make_line((100.5945, -41.8378, 0), (100.8741, -41.9496, 0)),
    Edge.make_line((100.8741, -41.9496, 0), (101.1538, -42.0615, 0)),
    Edge.make_line((101.1538, -42.0615, 0), (101.4334, -42.1733, 0)),
]
sk_Sketch18_loft_13_wire = Wire(sk_Sketch18_loft_13_edges)
sk_Sketch18_loft_13_face = Face(sk_Sketch18_loft_13_wire)
sk_Sketch18_loft_13_face = Plane(origin=Vector(0.0, 0.0, 16.3017), x_dir=Vector(-1.0, 0.0, 0.0), z_dir=Vector(0.0, 0.0, 1.0)) * sk_Sketch18_loft_13_face
_pl_14 = Plane(
    origin=Vector(0.1341, 0.1214, 0.5531),
    x_dir=Vector(-0.670666, 0.741759, 0.0),
    z_dir=Vector(0.230537, 0.208442, 0.950476),
)
with BuildSketch(_pl_14) as sk_Sketch19_14:
    with BuildLine():
        Line((99.6476, 46.462), (91.4358, 46.462))
        Line((91.4358, 46.462), (91.4358, 53.2407))
        Line((91.4358, 53.2407), (99.6476, 53.2407))
        Line((99.6476, 53.2407), (99.6476, 46.462))
    _edges_sk_Sketch19_14 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch19_14 = Wire.combine(_edges_sk_Sketch19_14)[0]
_w_sk_Sketch19_14 = _w_sk_Sketch19_14.moved(_pl_14.location)
_mf_sk_Sketch19_14 = BRepBuilderAPI_MakeFace(_pl_14.wrapped, _w_sk_Sketch19_14.wrapped, True)
_face_sk_Sketch19_14 = Face(_mf_sk_Sketch19_14.Face())
_pl_15 = Plane(
    origin=Vector(-30.5781, -0.0167, 52.9635),
    x_dir=Vector(0.000545, -1.0, 0.0),
    z_dir=Vector(-0.499995, -0.000273, 0.866028),
)
with BuildSketch(_pl_15) as sk_Sketch20_15:
    with Locations((-13.1288, -26.7932)):
        Circle(radius=21.0006)

_ext_sk_Sketch20_15 = extrude(sk_Sketch20_15.sketch, amount=-5.0, dir=Vector(-0.499995, -0.000273, 0.866028)).solid()
_pl_16 = Plane(
    origin=Vector(-23.2053, -0.0, -13.3991),
    x_dir=Vector(-0.500044, 0.000127, 0.866),
    z_dir=Vector(0.866, 0.0, 0.500044),
)
with BuildSketch(_pl_16) as sk_Sketch22_15:
    with BuildLine():
        Line((61.1646, -36.0916), (63.6085, -35.683))
        Line((63.6085, -35.683), (64.0, -24.0))
        Line((64.0, -24.0), (50.6559, -24.5621))
        Line((50.6559, -24.5621), (55.1674, -36.0924))
        Line((55.1674, -36.0924), (59.1664, -36.0919))
        Line((59.1664, -36.0919), (59.1661, -34.0919))
        Line((59.1661, -34.0919), (60.1671, -34.0917))
        Line((60.1671, -34.0917), (61.1646, -36.0916))
    make_face()
_pl_17 = Plane(
    origin=Vector(-30.5781, -0.0167, 52.9635),
    x_dir=Vector(0.000545, -1.0, 0.0),
    z_dir=Vector(-0.499995, -0.000273, 0.866028),
)
with BuildSketch(_pl_17) as sk_Sketch23_17:
    with BuildLine():
        Line((-37.09, -15.2), (-37.0919, -15.2239))
        Line((-37.0919, -15.2239), (-37.0555, -15.6558))
        Line((-37.0555, -15.6558), (-37.0158, -16.4578))
        Line((-37.0158, -16.4578), (-36.8262, -16.525))
        Line((-36.8262, -16.525), (-36.724, -15.6975))
        Line((-36.724, -15.6975), (-33.6781, -11.1653))
        Line((-33.6781, -11.1653), (-31.3143, -8.1632))
        Line((-31.3143, -8.1632), (-28.4364, -4.0679))
        Line((-28.4364, -4.0679), (-24.5623, -0.7727))
        Line((-24.5623, -0.7727), (-26.9986, -1.3121))
        Line((-26.9986, -1.3121), (-29.4269, -1.85))
        Line((-29.4269, -1.85), (-30.2228, -3.0142))
        Line((-30.2228, -3.0142), (-31.8308, -5.3662))
        Line((-31.8308, -5.3662), (-32.8182, -7.2126))
        Line((-32.8182, -7.2126), (-33.7285, -8.9147))
        Line((-33.7285, -8.9147), (-35.3488, -11.9446))
        Line((-35.3488, -11.9446), (-36.594, -14.2734))
        Line((-36.594, -14.2734), (-37.09, -15.2))
    _edges_sk_Sketch23_17 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch23_17 = Wire.combine(_edges_sk_Sketch23_17)[0]
_w_sk_Sketch23_17 = _w_sk_Sketch23_17.moved(_pl_17.location)
_mf_sk_Sketch23_17 = BRepBuilderAPI_MakeFace(_pl_17.wrapped, _w_sk_Sketch23_17.wrapped, True)
_face_sk_Sketch23_17 = Face(_mf_sk_Sketch23_17.Face())
_pl_18 = Plane(
    origin=Vector(-30.6025, 0.0135, 52.9699),
    x_dir=Vector(-0.00044, -1.0, 0.0),
    z_dir=Vector(-0.500249, 0.00022, 0.865882),
)
with BuildSketch(_pl_18) as sk_Sketch24_18:
    with BuildLine():
        Spline((-36.2033, -18.2041), (-36.1991, -17.9452), (-36.2251, -17.6876), (-36.2809, -17.4348), (-36.3659, -17.1902), (-36.4788, -16.9572), (-36.6181, -16.739), (-36.7819, -16.5385))
        Line((-36.7819, -16.5385), (-36.9716, -16.4715))
        Line((-36.9716, -16.4715), (-37.2446, -16.4852))
        Line((-37.2446, -16.4852), (-37.5434, -18.3881))
        Line((-37.5434, -18.3881), (-38.2248, -23.1301))
        Line((-38.2248, -23.1301), (-38.6435, -26.6543))
        Line((-38.6435, -26.6543), (-38.528, -27.7136))
        Line((-38.528, -27.7136), (-37.2112, -34.1126))
        Line((-37.2112, -34.1126), (-37.005, -29.5634))
        Line((-37.005, -29.5634), (-36.8618, -27.291))
        Line((-36.8618, -27.291), (-36.5436, -22.7462))
        Line((-36.5436, -22.7462), (-36.2033, -18.2041))
    _edges_sk_Sketch24_18 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch24_18 = Wire.combine(_edges_sk_Sketch24_18)[0]
_w_sk_Sketch24_18 = _w_sk_Sketch24_18.moved(_pl_18.location)
_mf_sk_Sketch24_18 = BRepBuilderAPI_MakeFace(_pl_18.wrapped, _w_sk_Sketch24_18.wrapped, True)
_face_sk_Sketch24_18 = Face(_mf_sk_Sketch24_18.Face())
_pl_19 = Plane(
    origin=Vector(-43.0091, -21.9621, -25.324),
    x_dir=Vector(0.454778, -0.890605, 0.0),
    z_dir=Vector(-0.788736, -0.402759, -0.464413),
)
with BuildSketch(_pl_19) as sk_Sketch31_19:
    with BuildLine():
        Line((-48.3261, 58.0626), (-47.7364, 62.0189))
        Line((-47.7364, 62.0189), (-66.5274, 64.8199))
        Line((-66.5274, 64.8199), (-67.1171, 60.8636))
        Line((-67.1171, 60.8636), (-48.3261, 58.0626))
    _edges_sk_Sketch31_19 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch31_19 = Wire.combine(_edges_sk_Sketch31_19)[0]
_w_sk_Sketch31_19 = _w_sk_Sketch31_19.moved(_pl_19.location)
_mf_sk_Sketch31_19 = BRepBuilderAPI_MakeFace(_pl_19.wrapped, _w_sk_Sketch31_19.wrapped, True)
_face_sk_Sketch31_19 = Face(_mf_sk_Sketch31_19.Face())
_surf_20 = Plane(
    origin=Vector(-30.8018, -22.1526, -23.7508),
    x_dir=Vector(-0.583875, 0.811843, 0.0),
    z_dir=Vector(0.688131, 0.494902, 0.53061),
)
with BuildSketch(_surf_20) as sk_Sketch32_20:
    with BuildLine():
        Line((78.1202, 58.9547), (69.8077, 58.669))
        Line((69.8077, 58.669), (67.7639, 62.5186))
        Line((67.7639, 62.5186), (77.9823, 62.9665))
        Line((77.9823, 62.9665), (78.1202, 58.9547))
    _edges_sk_Sketch32_20 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch32_20 = Wire.combine(_edges_sk_Sketch32_20)[0]
_w_sk_Sketch32_20 = _w_sk_Sketch32_20.moved(_surf_20.location)
_mf_sk_Sketch32_20 = BRepBuilderAPI_MakeFace(_surf_20.wrapped, _w_sk_Sketch32_20.wrapped, True)
_face_sk_Sketch32_20 = Face(_mf_sk_Sketch32_20.Face())
_surf_21 = Plane(
    origin=Vector(-23.0829, -18.9806, 47.6387),
    x_dir=Vector(-0.635133, 0.772403, 0.0),
    z_dir=Vector(0.410461, 0.337515, -0.847116),
)
with BuildSketch(_surf_21) as sk_Sketch33_21:
    with BuildLine():
        Line((82.3528, -42.0), (63.488, -43.6201))
        Line((63.488, -43.6201), (62.649, -41.3914))
        Line((62.649, -41.3914), (82.3528, -38.8726))
        Line((82.3528, -38.8726), (82.3528, -42.0))
    _edges_sk_Sketch33_21 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch33_21 = Wire.combine(_edges_sk_Sketch33_21)[0]
_w_sk_Sketch33_21 = _w_sk_Sketch33_21.moved(_surf_21.location)
_mf_sk_Sketch33_21 = BRepBuilderAPI_MakeFace(_surf_21.wrapped, _w_sk_Sketch33_21.wrapped, True)
_face_sk_Sketch33_21 = Face(_mf_sk_Sketch33_21.Face())
_surf_22 = Plane(
    origin=Vector(-18.5737, -18.4742, 47.5662),
    x_dir=Vector(0.705203, -0.709006, -0.0),
    z_dir=Vector(-0.342038, -0.340204, 0.87594),
)
with BuildSketch(_surf_22) as sk_Sketch34_22:
    with BuildLine():
        Line((-77.9886, -36.5671), (-78.1814, -37.9863))
        Line((-78.1814, -37.9863), (-80.6308, -36.2027))
        Line((-80.6308, -36.2027), (-77.9886, -36.5671))
    _edges_sk_Sketch34_22 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch34_22 = Wire.combine(_edges_sk_Sketch34_22)[0]
_w_sk_Sketch34_22 = _w_sk_Sketch34_22.moved(_surf_22.location)
_mf_sk_Sketch34_22 = BRepBuilderAPI_MakeFace(_surf_22.wrapped, _w_sk_Sketch34_22.wrapped, True)
_face_sk_Sketch34_22 = Face(_mf_sk_Sketch34_22.Face())
_surf_23 = Plane(
    origin=Vector(-24.5641, -20.9303, 50.3967),
    x_dir=Vector(0.648559, -0.761165, 0.0),
    z_dir=Vector(-0.410471, -0.349747, 0.842135),
)
with BuildSketch(_surf_23) as sk_Sketch35_23:
    with BuildLine():
        Line((-68.3251, -44.9238), (-86.2385, -44.9238))
        Line((-86.2385, -44.9238), (-86.2385, -30.5686))
        Line((-86.2385, -30.5686), (-68.3251, -30.5686))
        Line((-68.3251, -30.5686), (-68.3251, -44.9238))
    _edges_sk_Sketch35_23 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch35_23 = Wire.combine(_edges_sk_Sketch35_23)[0]
_w_sk_Sketch35_23 = _w_sk_Sketch35_23.moved(_surf_23.location)
_mf_sk_Sketch35_23 = BRepBuilderAPI_MakeFace(_surf_23.wrapped, _w_sk_Sketch35_23.wrapped, True)
_face_sk_Sketch35_23 = Face(_mf_sk_Sketch35_23.Face())
_surf_24 = Plane(
    origin=Vector(-8.6941, -10.4743, 42.0437),
    x_dir=Vector(0.769466, -0.638688, 0.0),
    z_dir=Vector(-0.196733, -0.237016, 0.951378),
)
with BuildSketch(_surf_24) as sk_Sketch37_24:
    with BuildLine():
        Line((-87.8366, -36.9185), (-83.3143, -40.2782))
        Line((-83.3143, -40.2782), (-78.738, -41.3579))
        Line((-78.738, -41.3579), (-73.6545, -42.5614))
        Line((-73.6545, -42.5614), (-74.2971, -40.3377))
        Line((-74.2971, -40.3377), (-87.8366, -36.9185))
    _edges_sk_Sketch37_24 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch37_24 = Wire.combine(_edges_sk_Sketch37_24)[0]
_w_sk_Sketch37_24 = _w_sk_Sketch37_24.moved(_surf_24.location)
_mf_sk_Sketch37_24 = BRepBuilderAPI_MakeFace(_surf_24.wrapped, _w_sk_Sketch37_24.wrapped, True)
_face_sk_Sketch37_24 = Face(_mf_sk_Sketch37_24.Face())
_surf_25 = Plane(
    origin=Vector(-24.5642, -20.9302, 50.3967),
    x_dir=Vector(0.648559, -0.761165, -0.0),
    z_dir=Vector(-0.410471, -0.349747, 0.842135),
)
with BuildSketch(_surf_25) as sk_Sketch38_25:
    with BuildLine():
        Line((-51.2713, -46.1116), (-92.699, -46.1116))
        Line((-92.699, -46.1116), (-92.699, -33.7995))
        Line((-92.699, -33.7995), (-51.2713, -33.7995))
        Line((-51.2713, -33.7995), (-51.2713, -46.1116))
    _edges_sk_Sketch38_25 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch38_25 = Wire.combine(_edges_sk_Sketch38_25)[0]
_w_sk_Sketch38_25 = _w_sk_Sketch38_25.moved(_surf_25.location)
_mf_sk_Sketch38_25 = BRepBuilderAPI_MakeFace(_surf_25.wrapped, _w_sk_Sketch38_25.wrapped, True)
_face_sk_Sketch38_25 = Face(_mf_sk_Sketch38_25.Face())
with BuildLine() as _bl_Sweep2:
    ThreePointArc((-80.0234, -4.4933, 10.3778), (-79.2374, -1.9039, 20.1316), (-74.03, 2.0048, 27.8818))
path_Sweep2 = _bl_Sweep2.wires()[0]
_plane_Sweep2 = Plane(origin=Vector(-13.5191, -13.094, -14.7438), x_dir=Vector(0.49080198, -0.82434858, 0.28206887), z_dir=Vector(0.56546095, 0.54767795, 0.61668694))
with BuildSketch(_plane_Sweep2) as sk_Sketch41_25:
    with BuildLine():
        Line((-40.2927, -68.316), (-40.2899, -68.3358))
        RadiusArc((-40.2899, -68.3358), (-40.1626, -68.7295), -1.0462)
        RadiusArc((-40.1626, -68.7295), (-40.1205, -69.1464), -1.061)
        Line((-40.1205, -69.1464), (-40.0344, -69.4258))
        Line((-40.0344, -69.4258), (-39.2818, -69.6255))
        Line((-39.2818, -69.6255), (-39.0071, -69.3488))
        Line((-39.0071, -69.3488), (-18.4013, -69.7991))
        Line((-18.4013, -69.7991), (-16.5896, -69.7975))
        RadiusArc((-16.5896, -69.7975), (-14.6574, -69.2999), -3.5549)
        RadiusArc((-14.6574, -69.2999), (-12.7445, -67.2971), -2.5558)
        Line((-12.7445, -67.2971), (-11.7998, -62.3813))
        Line((-11.7998, -62.3813), (-26.392, -62.738))
        Line((-26.392, -62.738), (-40.8823, -62.3869))
        Line((-40.8823, -62.3869), (-40.2927, -68.316))
    make_face()
sk_Sketch53_loft_27_edges = [
    Edge.make_line((-118.3529, 10.6108, 0), (-118.2966, 9.9176, 0)),
    Edge.make_line((-118.2966, 9.9176, 0), (-118.2402, 9.2244, 0)),
    Edge.make_line((-118.2402, 9.2244, 0), (-118.0295, 8.5833, 0)),
    Edge.make_line((-118.0295, 8.5833, 0), (-117.6546, 7.9976, 0)),
    Edge.make_line((-117.6546, 7.9976, 0), (-117.2796, 7.4119, 0)),
    Edge.make_line((-117.2796, 7.4119, 0), (-116.8539, 6.862, 0)),
    Edge.make_line((-116.8539, 6.862, 0), (-116.4275, 6.3126, 0)),
    Edge.make_line((-116.4275, 6.3126, 0), (-116.001, 5.7632, 0)),
    Edge.make_line((-116.001, 5.7632, 0), (-115.5746, 5.2138, 0)),
    Edge.make_line((-115.5746, 5.2138, 0), (-115.1481, 4.6645, 0)),
    Edge.make_line((-115.1481, 4.6645, 0), (-114.7217, 4.1151, 0)),
    Edge.make_line((-114.7217, 4.1151, 0), (-114.2952, 3.5657, 0)),
    Edge.make_line((-114.2952, 3.5657, 0), (-113.8688, 3.0163, 0)),
    Edge.make_line((-113.8688, 3.0163, 0), (-113.4423, 2.467, 0)),
    Edge.make_line((-113.4423, 2.467, 0), (-113.0159, 1.9176, 0)),
    Edge.make_line((-113.0159, 1.9176, 0), (-112.5894, 1.3682, 0)),
    Edge.make_line((-112.5894, 1.3682, 0), (-112.163, 0.8189, 0)),
    Edge.make_line((-112.163, 0.8189, 0), (-111.7365, 0.2695, 0)),
    Edge.make_line((-111.7365, 0.2695, 0), (-111.3101, -0.2799, 0)),
    Edge.make_line((-111.3101, -0.2799, 0), (-110.8836, -0.8293, 0)),
    Edge.make_line((-110.8836, -0.8293, 0), (-110.4572, -1.3786, 0)),
    Edge.make_line((-110.4572, -1.3786, 0), (-110.0307, -1.928, 0)),
    Edge.make_line((-110.0307, -1.928, 0), (-109.6043, -2.4774, 0)),
    Edge.make_line((-109.6043, -2.4774, 0), (-109.1778, -3.0268, 0)),
    Edge.make_line((-109.1778, -3.0268, 0), (-108.7514, -3.5761, 0)),
    Edge.make_line((-108.7514, -3.5761, 0), (-108.2431, -4.0424, 0)),
    Edge.make_line((-108.2431, -4.0424, 0), (-107.6872, -4.4603, 0)),
    Edge.make_line((-107.6872, -4.4603, 0), (-107.1312, -4.8781, 0)),
    Edge.make_line((-107.1312, -4.8781, 0), (-106.5752, -5.2959, 0)),
    Edge.make_line((-106.5752, -5.2959, 0), (-106.0163, -5.2115, 0)),
    Edge.make_line((-106.0163, -5.2115, 0), (-105.4554, -4.8003, 0)),
    Edge.make_line((-105.4554, -4.8003, 0), (-104.8946, -4.389, 0)),
    Edge.make_line((-104.8946, -4.389, 0), (-104.3337, -3.9778, 0)),
    Edge.make_line((-104.3337, -3.9778, 0), (-104.896, -3.8099, 0)),
    Edge.make_line((-104.896, -3.8099, 0), (-105.5768, -3.6676, 0)),
    Edge.make_line((-105.5768, -3.6676, 0), (-106.1661, -3.3498, 0)),
    Edge.make_line((-106.1661, -3.3498, 0), (-106.6728, -2.8735, 0)),
    Edge.make_line((-106.6728, -2.8735, 0), (-107.1536, -2.3736, 0)),
    Edge.make_line((-107.1536, -2.3736, 0), (-107.5801, -1.8242, 0)),
    Edge.make_line((-107.5801, -1.8242, 0), (-108.0066, -1.2749, 0)),
    Edge.make_line((-108.0066, -1.2749, 0), (-108.4331, -0.7256, 0)),
    Edge.make_line((-108.4331, -0.7256, 0), (-108.8596, -0.1762, 0)),
    Edge.make_line((-108.8596, -0.1762, 0), (-109.2861, 0.3731, 0)),
    Edge.make_line((-109.2861, 0.3731, 0), (-109.7126, 0.9225, 0)),
    Edge.make_line((-109.7126, 0.9225, 0), (-110.1391, 1.4718, 0)),
    Edge.make_line((-110.1391, 1.4718, 0), (-110.5656, 2.0211, 0)),
    Edge.make_line((-110.5656, 2.0211, 0), (-110.9921, 2.5705, 0)),
    Edge.make_line((-110.9921, 2.5705, 0), (-111.4186, 3.1198, 0)),
    Edge.make_line((-111.4186, 3.1198, 0), (-111.8451, 3.6691, 0)),
    Edge.make_line((-111.8451, 3.6691, 0), (-112.2716, 4.2185, 0)),
    Edge.make_line((-112.2716, 4.2185, 0), (-112.6981, 4.7678, 0)),
    Edge.make_line((-112.6981, 4.7678, 0), (-113.1246, 5.3171, 0)),
    Edge.make_line((-113.1246, 5.3171, 0), (-113.5511, 5.8665, 0)),
    Edge.make_line((-113.5511, 5.8665, 0), (-113.9776, 6.4158, 0)),
    Edge.make_line((-113.9776, 6.4158, 0), (-114.4041, 6.9651, 0)),
    Edge.make_line((-114.4041, 6.9651, 0), (-114.8306, 7.5145, 0)),
    Edge.make_line((-114.8306, 7.5145, 0), (-115.2571, 8.0638, 0)),
    Edge.make_line((-115.2571, 8.0638, 0), (-115.6836, 8.6132, 0)),
    Edge.make_line((-115.6836, 8.6132, 0), (-116.0009, 9.2316, 0)),
    Edge.make_line((-116.0009, 9.2316, 0), (-116.3149, 9.8522, 0)),
    Edge.make_line((-116.3149, 9.8522, 0), (-116.6288, 10.4727, 0)),
    Edge.make_line((-116.6288, 10.4727, 0), (-117.0053, 10.9554, 0)),
    Edge.make_line((-117.0053, 10.9554, 0), (-117.6791, 10.7831, 0)),
    Edge.make_line((-117.6791, 10.7831, 0), (-118.3529, 10.6108, 0)),
]
sk_Sketch53_loft_27_wire = Wire(sk_Sketch53_loft_27_edges)
sk_Sketch53_loft_27_face = Face(sk_Sketch53_loft_27_wire)
sk_Sketch53_loft_27_face = Plane(origin=Vector(0.0, 0.0, 10.9074), x_dir=Vector(1.0, 0.0, 0.0), z_dir=Vector(0.0, 0.0, 1.0)) * sk_Sketch53_loft_27_face
sk_Sketch52_loft_28_edges = [
    Edge.make_line((59.2278, -72.5506, 0), (59.1452, -73.2841, 0)),
    Edge.make_line((59.1452, -73.2841, 0), (59.0626, -74.0176, 0)),
    Edge.make_line((59.0626, -74.0176, 0), (58.98, -74.7511, 0)),
    Edge.make_line((58.98, -74.7511, 0), (58.8974, -75.4846, 0)),
    Edge.make_line((58.8974, -75.4846, 0), (58.8148, -76.2181, 0)),
    Edge.make_line((58.8148, -76.2181, 0), (58.7637, -76.9236, 0)),
    Edge.make_line((58.7637, -76.9236, 0), (59.5018, -76.9268, 0)),
    Edge.make_line((59.5018, -76.9268, 0), (60.24, -76.93, 0)),
    Edge.make_line((60.24, -76.93, 0), (60.9781, -76.9332, 0)),
    Edge.make_line((60.9781, -76.9332, 0), (61.7163, -76.9363, 0)),
    Edge.make_line((61.7163, -76.9363, 0), (62.4544, -76.9395, 0)),
    Edge.make_line((62.4544, -76.9395, 0), (63.1925, -76.9427, 0)),
    Edge.make_line((63.1925, -76.9427, 0), (63.9307, -76.9459, 0)),
    Edge.make_line((63.9307, -76.9459, 0), (64.6688, -76.9491, 0)),
    Edge.make_line((64.6688, -76.9491, 0), (65.4069, -76.9522, 0)),
    Edge.make_line((65.4069, -76.9522, 0), (66.1451, -76.9554, 0)),
    Edge.make_line((66.1451, -76.9554, 0), (66.8832, -76.9586, 0)),
    Edge.make_line((66.8832, -76.9586, 0), (67.6213, -76.9618, 0)),
    Edge.make_line((67.6213, -76.9618, 0), (68.3595, -76.965, 0)),
    Edge.make_line((68.3595, -76.965, 0), (69.0976, -76.9681, 0)),
    Edge.make_line((69.0976, -76.9681, 0), (69.8357, -76.9713, 0)),
    Edge.make_line((69.8357, -76.9713, 0), (70.5739, -76.9745, 0)),
    Edge.make_line((70.5739, -76.9745, 0), (71.312, -76.9777, 0)),
    Edge.make_line((71.312, -76.9777, 0), (72.0501, -76.9809, 0)),
    Edge.make_line((72.0501, -76.9809, 0), (72.7883, -76.984, 0)),
    Edge.make_line((72.7883, -76.984, 0), (73.5264, -76.9872, 0)),
    Edge.make_line((73.5264, -76.9872, 0), (74.2645, -76.9904, 0)),
    Edge.make_line((74.2645, -76.9904, 0), (75.0027, -76.9936, 0)),
    Edge.make_line((75.0027, -76.9936, 0), (75.7408, -76.9968, 0)),
    Edge.make_line((75.7408, -76.9968, 0), (76.479, -76.9999, 0)),
    Edge.make_line((76.479, -76.9999, 0), (77.2171, -77.0031, 0)),
    Edge.make_line((77.2171, -77.0031, 0), (77.9552, -77.0063, 0)),
    Edge.make_line((77.9552, -77.0063, 0), (78.1099, -76.3653, 0)),
    Edge.make_line((78.1099, -76.3653, 0), (78.1795, -75.6305, 0)),
    Edge.make_line((78.1795, -75.6305, 0), (78.2492, -74.8956, 0)),
    Edge.make_line((78.2492, -74.8956, 0), (78.3188, -74.1608, 0)),
    Edge.make_line((78.3188, -74.1608, 0), (78.3884, -73.4259, 0)),
    Edge.make_line((78.3884, -73.4259, 0), (78.2786, -72.8541, 0)),
    Edge.make_line((78.2786, -72.8541, 0), (77.5405, -72.8532, 0)),
    Edge.make_line((77.5405, -72.8532, 0), (76.8023, -72.8523, 0)),
    Edge.make_line((76.8023, -72.8523, 0), (76.0642, -72.8514, 0)),
    Edge.make_line((76.0642, -72.8514, 0), (75.3261, -72.8505, 0)),
    Edge.make_line((75.3261, -72.8505, 0), (74.5879, -72.8496, 0)),
    Edge.make_line((74.5879, -72.8496, 0), (73.8498, -72.8487, 0)),
    Edge.make_line((73.8498, -72.8487, 0), (73.1116, -72.8478, 0)),
    Edge.make_line((73.1116, -72.8478, 0), (72.3735, -72.8469, 0)),
    Edge.make_line((72.3735, -72.8469, 0), (71.6354, -72.846, 0)),
    Edge.make_line((71.6354, -72.846, 0), (70.8972, -72.845, 0)),
    Edge.make_line((70.8972, -72.845, 0), (70.1591, -72.8441, 0)),
    Edge.make_line((70.1591, -72.8441, 0), (69.4209, -72.8432, 0)),
    Edge.make_line((69.4209, -72.8432, 0), (68.6828, -72.8423, 0)),
    Edge.make_line((68.6828, -72.8423, 0), (67.9447, -72.8414, 0)),
    Edge.make_line((67.9447, -72.8414, 0), (67.2065, -72.8405, 0)),
    Edge.make_line((67.2065, -72.8405, 0), (66.4684, -72.8396, 0)),
    Edge.make_line((66.4684, -72.8396, 0), (65.7302, -72.8387, 0)),
    Edge.make_line((65.7302, -72.8387, 0), (64.9921, -72.8378, 0)),
    Edge.make_line((64.9921, -72.8378, 0), (64.2539, -72.8369, 0)),
    Edge.make_line((64.2539, -72.8369, 0), (63.5158, -72.836, 0)),
    Edge.make_line((63.5158, -72.836, 0), (62.7777, -72.8351, 0)),
    Edge.make_line((62.7777, -72.8351, 0), (62.0395, -72.8342, 0)),
    Edge.make_line((62.0395, -72.8342, 0), (61.3014, -72.8333, 0)),
    Edge.make_line((61.3014, -72.8333, 0), (60.5632, -72.8324, 0)),
    Edge.make_line((60.5632, -72.8324, 0), (59.8251, -72.8315, 0)),
    Edge.make_line((59.8251, -72.8315, 0), (59.2278, -72.5506, 0)),
]
sk_Sketch52_loft_28_wire = Wire(sk_Sketch52_loft_28_edges)
sk_Sketch52_loft_28_face = Face(sk_Sketch52_loft_28_wire)
sk_Sketch52_loft_28_face = Plane(origin=Vector(-22.9442, -14.6141, -32.3455), x_dir=Vector(-0.579545, 0.813783, 0.043421), z_dir=Vector(-0.542879, -0.345782, -0.765322)) * sk_Sketch52_loft_28_face
_surf_29 = Plane(
    origin=Vector(-51.6877, 66.5376, 0.0001),
    x_dir=Vector(-1e-06, 0.0, -1.0),
    z_dir=Vector(0.613469, -0.789719, -1e-06),
)
with BuildSketch(_surf_29) as sk_Sketch58_29:
    with BuildLine():
        RadiusArc((-16.1895, -69.3133), (-14.4998, -71.5), -19.0465)
        Line((-14.4998, -71.5), (-13.8976, -73.75))
        RadiusArc((-13.8976, -73.75), (-12.969, -75.4551), -34.7089)
        Line((-12.969, -75.4551), (-6.9072, -75.4551))
        Line((-6.9072, -75.4551), (-6.9072, -63.7312))
        Line((-6.9072, -63.7312), (-20.9072, -63.5455))
        RadiusArc((-20.9072, -63.5455), (-16.1895, -69.3133), 37.1981)
    _edges_sk_Sketch58_29 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch58_29 = Wire.combine(_edges_sk_Sketch58_29)[0]
_w_sk_Sketch58_29 = _w_sk_Sketch58_29.moved(_surf_29.location)
_mf_sk_Sketch58_29 = BRepBuilderAPI_MakeFace(_surf_29.wrapped, _w_sk_Sketch58_29.wrapped, True)
_face_sk_Sketch58_29 = Face(_mf_sk_Sketch58_29.Face())
_ref_plane_30 = Plane(
    origin=Vector(-97.745, -7.0104, 0.0),
    x_dir=Vector(0.0, 0.0, 1.0),
    z_dir=Vector(-0.997438, -0.071538, 0.0),
)
with BuildSketch(_ref_plane_30) as sk_Sketch59_30:
    with BuildLine():
        Line((16.1122, 46.0055), (17.5025, 44.3353))
        RadiusArc((17.5025, 44.3353), (19.6065, 42.4984), -6.6348)
        RadiusArc((19.6065, 42.4984), (20.9967, 41.4533), -35.549)
        Line((20.9967, 41.4533), (6.8597, 41.4533))
        Line((6.8597, 41.4533), (6.8597, 42.6697))
        Line((6.8597, 42.6697), (16.154, 42.888))
        Line((16.154, 42.888), (16.1122, 46.0055))
    _edges_sk_Sketch59_30 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch59_30 = Wire.combine(_edges_sk_Sketch59_30)[0]
_w_sk_Sketch59_30 = _w_sk_Sketch59_30.moved(_ref_plane_30.location)
_mf_sk_Sketch59_30 = BRepBuilderAPI_MakeFace(_ref_plane_30.wrapped, _w_sk_Sketch59_30.wrapped, True)
_face_sk_Sketch59_30 = Face(_mf_sk_Sketch59_30.Face())
_ref_plane_31 = Plane(
    origin=Vector(-103.4562, 19.102, 0.0),
    x_dir=Vector(0.0, 0.0, 1.0),
    z_dir=Vector(-0.983378, 0.18157, 0.0),
)
with BuildSketch(_ref_plane_31) as sk_Sketch60_31:
    with BuildLine():
        Line((6.9074, 12.5052), (6.9074, 14.4515))
        Line((6.9074, 14.4515), (22.0, 14.7077))
        Line((22.0, 14.7077), (22.7801, 14.4515))
        Line((22.7801, 14.4515), (25.7323, 13.4479))
        Line((25.7323, 13.4479), (26.3484, 13.2288))
        Line((26.3484, 13.2288), (6.9074, 12.5052))
    _edges_sk_Sketch60_31 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch60_31 = Wire.combine(_edges_sk_Sketch60_31)[0]
_w_sk_Sketch60_31 = _w_sk_Sketch60_31.moved(_ref_plane_31.location)
_mf_sk_Sketch60_31 = BRepBuilderAPI_MakeFace(_ref_plane_31.wrapped, _w_sk_Sketch60_31.wrapped, True)
_face_sk_Sketch60_31 = Face(_mf_sk_Sketch60_31.Face())
_ref_plane_32 = Plane(
    origin=Vector(-102.5633, 6.6616, 0.0),
    x_dir=Vector(0.0, 0.0, 1.0),
    z_dir=Vector(-0.997897, 0.064814, 0.0),
)
with BuildSketch(_ref_plane_32) as sk_Sketch61_32:
    with BuildLine():
        Line((20.8348, 27.9967), (6.9074, 28.5342))
        Line((6.9074, 28.5342), (6.9074, 25.941))
        Line((6.9074, 25.941), (22.4963, 26.1468))
        RadiusArc((22.4963, 26.1468), (20.8348, 27.9967), 1.9197)
    _edges_sk_Sketch61_32 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch61_32 = Wire.combine(_edges_sk_Sketch61_32)[0]
_w_sk_Sketch61_32 = _w_sk_Sketch61_32.moved(_ref_plane_32.location)
_mf_sk_Sketch61_32 = BRepBuilderAPI_MakeFace(_ref_plane_32.wrapped, _w_sk_Sketch61_32.wrapped, True)
_face_sk_Sketch61_32 = Face(_mf_sk_Sketch61_32.Face())
_ref_plane_33 = Plane(
    origin=Vector(-102.5633, 6.6616, 0.0),
    x_dir=Vector(-0.064814, -0.997897, 0.0),
    z_dir=Vector(-0.997897, 0.064814, 0.0),
)
with BuildSketch(_ref_plane_33) as sk_Sketch62_33:
    with BuildLine():
        RadiusArc((-27.9967, 20.8348), (-26.7079, 21.3804), -1.9197)
        Line((-26.7079, 21.3804), (-26.9835, 22.0959))
        RadiusArc((-26.9835, 22.0959), (-28.0418, 20.8049), 2.9475)
        Line((-28.0418, 20.8049), (-27.9967, 20.8348))
    _edges_sk_Sketch62_33 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch62_33 = Wire.combine(_edges_sk_Sketch62_33)[0]
_w_sk_Sketch62_33 = _w_sk_Sketch62_33.moved(_ref_plane_33.location)
_mf_sk_Sketch62_33 = BRepBuilderAPI_MakeFace(_ref_plane_33.wrapped, _w_sk_Sketch62_33.wrapped, True)
_face_sk_Sketch62_33 = Face(_mf_sk_Sketch62_33.Face())
_ref_plane_34 = Plane(
    origin=Vector(-89.5885, 48.7912, 0.0),
    x_dir=Vector(0.0, 0.0, -1.0),
    z_dir=Vector(0.878205, -0.478284, 0.0),
)
with BuildSketch(_ref_plane_34) as sk_Sketch63_34:
    with BuildLine():
        Line((-21.2005, -20.546), (-21.2025, -20.6852))
        Line((-21.2025, -20.6852), (-21.2025, -22.2997))
        Line((-21.2025, -22.2997), (-10.6374, -22.4525))
        Line((-10.6374, -22.4525), (-10.612, -20.6992))
        Line((-10.612, -20.6992), (-21.2005, -20.546))
    _edges_sk_Sketch63_34 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch63_34 = Wire.combine(_edges_sk_Sketch63_34)[0]
_w_sk_Sketch63_34 = _w_sk_Sketch63_34.moved(_ref_plane_34.location)
_mf_sk_Sketch63_34 = BRepBuilderAPI_MakeFace(_ref_plane_34.wrapped, _w_sk_Sketch63_34.wrapped, True)
_face_sk_Sketch63_34 = Face(_mf_sk_Sketch63_34.Face())
_ref_plane_35 = Plane(
    origin=Vector(-98.4039, 33.7717, 0.0),
    x_dir=Vector(0.0, 0.0, -1.0),
    z_dir=Vector(0.945848, -0.32461, 0.0),
)
with BuildSketch(_ref_plane_35) as sk_Sketch66_35:
    with BuildLine():
        Line((-10.5376, -2.4503), (-10.5531, -3.5482))
        Line((-10.5531, -3.5482), (-21.2, -3.2557))
        Line((-21.2, -3.2557), (-21.2, -2.3))
        Line((-21.2, -2.3), (-10.5376, -2.4503))
    _edges_sk_Sketch66_35 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch66_35 = Wire.combine(_edges_sk_Sketch66_35)[0]
_w_sk_Sketch66_35 = _w_sk_Sketch66_35.moved(_ref_plane_35.location)
_mf_sk_Sketch66_35 = BRepBuilderAPI_MakeFace(_ref_plane_35.wrapped, _w_sk_Sketch66_35.wrapped, True)
_face_sk_Sketch66_35 = Face(_mf_sk_Sketch66_35.Face())
_ref_plane_36 = Plane(
    origin=Vector(0.0, 0.0, 6.9074),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_ref_plane_36) as sk_Sketch67_36:
    with BuildLine():
        Line((-61.4968, -6.9957), (-96.4334, -6.9957))
        Line((-96.4334, -6.9957), (-96.4334, 32.357))
        Line((-96.4334, 32.357), (-61.4968, 32.357))
        Line((-61.4968, 32.357), (-61.4968, -6.9957))
    _edges_sk_Sketch67_36 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch67_36 = Wire.combine(_edges_sk_Sketch67_36)[0]
_w_sk_Sketch67_36 = _w_sk_Sketch67_36.moved(_ref_plane_36.location)
_mf_sk_Sketch67_36 = BRepBuilderAPI_MakeFace(_ref_plane_36.wrapped, _w_sk_Sketch67_36.wrapped, True)
_face_sk_Sketch67_36 = Face(_mf_sk_Sketch67_36.Face())
_ref_plane_37 = Plane(
    origin=Vector(-54.1594, -26.5847, -4.1478),
    x_dir=Vector(0.386412, -0.844919, 0.369862),
    z_dir=Vector(0.89557, 0.439601, 0.068587),
)
with BuildSketch(_ref_plane_37) as sk_Sketch69_37:
    with BuildLine():
        RadiusArc((-14.4751, -18.2397), (-5.6735, -33.6476), -32.3482)
        RadiusArc((-5.6735, -33.6476), (-0.0687, -32.3275), -9.5164)
        RadiusArc((-0.0687, -32.3275), (-1.081, -12.7235), -42.7406)
        RadiusArc((-1.081, -12.7235), (-14.4751, -18.2397), -72.9661)
    _edges_sk_Sketch69_37 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch69_37 = Wire.combine(_edges_sk_Sketch69_37)[0]
_w_sk_Sketch69_37 = _w_sk_Sketch69_37.moved(_ref_plane_37.location)
_mf_sk_Sketch69_37 = BRepBuilderAPI_MakeFace(_ref_plane_37.wrapped, _w_sk_Sketch69_37.wrapped, True)
_face_sk_Sketch69_37 = Face(_mf_sk_Sketch69_37.Face())
_ref_plane_38 = Plane(
    origin=Vector(-59.3198, 65.884, 0.0),
    x_dir=Vector(0.0, 0.0, -1.0),
    z_dir=Vector(0.669115, -0.743159, 0.0),
)
with BuildSketch(_ref_plane_38) as sk_Sketch70_38:
    with BuildLine():
        Line((-10.7341, -54.3544), (-10.7341, -58.0))
        Line((-10.7341, -58.0), (-11.0, -58.0))
        Line((-11.0, -58.0), (-20.9581, -57.6446))
        Line((-20.9581, -57.6446), (-20.9581, -54.3544))
        Line((-20.9581, -54.3544), (-10.7341, -54.3544))
    _edges_sk_Sketch70_38 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch70_38 = Wire.combine(_edges_sk_Sketch70_38)[0]
_w_sk_Sketch70_38 = _w_sk_Sketch70_38.moved(_ref_plane_38.location)
_mf_sk_Sketch70_38 = BRepBuilderAPI_MakeFace(_ref_plane_38.wrapped, _w_sk_Sketch70_38.wrapped, True)
_face_sk_Sketch70_38 = Face(_mf_sk_Sketch70_38.Face())
_ref_plane_39 = Plane(
    origin=Vector(-115.8778, 16.4497, 0.0),
    x_dir=Vector(-0.0, 0.0, -1.0),
    z_dir=Vector(0.990074, -0.140548, -0.0),
)
with BuildSketch(_ref_plane_39) as sk_Sketch71_39:
    with BuildLine():
        Line((-11.1389, -4.7185), (-10.9001, -4.7185))
        Line((-10.9001, -4.7185), (-10.9001, -2.424))
        Line((-10.9001, -2.424), (-12.8099, -2.424))
        RadiusArc((-12.8099, -2.424), (-12.1624, -3.0663), 7.2097)
        RadiusArc((-12.1624, -3.0663), (-11.6214, -3.7558), 1.2766)
        RadiusArc((-11.6214, -3.7558), (-11.1389, -4.7185), 3.9383)
    _edges_sk_Sketch71_39 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch71_39 = Wire.combine(_edges_sk_Sketch71_39)[0]
_w_sk_Sketch71_39 = _w_sk_Sketch71_39.moved(_ref_plane_39.location)
_mf_sk_Sketch71_39 = BRepBuilderAPI_MakeFace(_ref_plane_39.wrapped, _w_sk_Sketch71_39.wrapped, True)
_face_sk_Sketch71_39 = Face(_mf_sk_Sketch71_39.Face())
_work_plane_40 = Plane(
    origin=Vector(-114.7258, -9.3256, 0.0),
    x_dir=Vector(0.0, 0.0, -1.0),
    z_dir=Vector(0.996713, 0.081019, 0.0),
)
with BuildSketch(_work_plane_40) as sk_Sketch72_40:
    with BuildLine():
        RadiusArc((-11.2, 21.7), (-11.1371, 21.2185), -0.7135)
        Line((-11.1371, 21.2185), (-11.0245, 20.9407))
        Line((-11.0245, 20.9407), (-10.9219, 20.032))
        Line((-10.9219, 20.032), (-10.8711, 21.7))
        Line((-10.8711, 21.7), (-11.2, 21.7))
    _edges_sk_Sketch72_40 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch72_40 = Wire.combine(_edges_sk_Sketch72_40)[0]
_w_sk_Sketch72_40 = _w_sk_Sketch72_40.moved(_work_plane_40.location)
_mf_sk_Sketch72_40 = BRepBuilderAPI_MakeFace(_work_plane_40.wrapped, _w_sk_Sketch72_40.wrapped, True)
_face_sk_Sketch72_40 = Face(_mf_sk_Sketch72_40.Face())
_work_plane_41 = Plane(
    origin=Vector(0.0, 0.0, 10.9074),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_work_plane_41) as sk_Sketch73_41:
    with BuildLine():
        RadiusArc((112.5316, -21.8054), (118.1412, -14.3227), -13.7823)
        Line((118.1412, -14.3227), (116.0993, -14.0328))
        RadiusArc((116.0993, -14.0328), (111.2623, -20.2314), 14.4758)
        Line((111.2623, -20.2314), (112.5316, -21.8054))
    _edges_sk_Sketch73_41 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch73_41 = Wire.combine(_edges_sk_Sketch73_41)[0]
_w_sk_Sketch73_41 = _w_sk_Sketch73_41.moved(_work_plane_41.location)
_mf_sk_Sketch73_41 = BRepBuilderAPI_MakeFace(_work_plane_41.wrapped, _w_sk_Sketch73_41.wrapped, True)
_face_sk_Sketch73_41 = Face(_mf_sk_Sketch73_41.Face())
sk_Sketch77_loft_42_edges = [
    Edge.make_line((-105.1759, -5.6165, 0), (-104.5184, -5.6172, 0)),
    Edge.make_line((-104.5184, -5.6172, 0), (-103.8619, -5.6016, 0)),
    Edge.make_line((-103.8619, -5.6016, 0), (-103.2099, -5.5169, 0)),
    Edge.make_line((-103.2099, -5.5169, 0), (-102.5641, -5.4003, 0)),
    Edge.make_line((-102.5641, -5.4003, 0), (-101.9286, -5.2315, 0)),
    Edge.make_line((-101.9286, -5.2315, 0), (-101.3086, -5.0167, 0)),
    Edge.make_line((-101.3086, -5.0167, 0), (-100.7004, -4.7668, 0)),
    Edge.make_line((-100.7004, -4.7668, 0), (-100.1207, -4.4586, 0)),
    Edge.make_line((-100.1207, -4.4586, 0), (-99.5501, -4.1318, 0)),
    Edge.make_line((-99.5501, -4.1318, 0), (-98.977, -3.8096, 0)),
    Edge.make_line((-98.977, -3.8096, 0), (-98.4037, -3.4876, 0)),
    Edge.make_line((-98.4037, -3.4876, 0), (-97.8304, -3.1656, 0)),
    Edge.make_line((-97.8304, -3.1656, 0), (-97.2571, -2.8437, 0)),
    Edge.make_line((-97.2571, -2.8437, 0), (-96.6838, -2.5217, 0)),
    Edge.make_line((-96.6838, -2.5217, 0), (-96.1105, -2.1997, 0)),
    Edge.make_line((-96.1105, -2.1997, 0), (-95.5372, -1.8777, 0)),
    Edge.make_line((-95.5372, -1.8777, 0), (-94.9639, -1.5557, 0)),
    Edge.make_line((-94.9639, -1.5557, 0), (-94.3906, -1.2338, 0)),
    Edge.make_line((-94.3906, -1.2338, 0), (-93.8173, -0.9118, 0)),
    Edge.make_line((-93.8173, -0.9118, 0), (-93.244, -0.5898, 0)),
    Edge.make_line((-93.244, -0.5898, 0), (-92.6707, -0.2678, 0)),
    Edge.make_line((-92.6707, -0.2678, 0), (-92.0975, 0.0542, 0)),
    Edge.make_line((-92.0975, 0.0542, 0), (-91.4934, 0.3031, 0)),
    Edge.make_line((-91.4934, 0.3031, 0), (-90.8622, 0.4872, 0)),
    Edge.make_line((-90.8622, 0.4872, 0), (-90.231, 0.6714, 0)),
    Edge.make_line((-90.231, 0.6714, 0), (-89.5998, 0.8556, 0)),
    Edge.make_line((-89.5998, 0.8556, 0), (-89.0608, 1.0996, 0)),
    Edge.make_line((-89.0608, 1.0996, 0), (-89.15, 1.751, 0)),
    Edge.make_line((-89.15, 1.751, 0), (-89.2391, 2.4025, 0)),
    Edge.make_line((-89.2391, 2.4025, 0), (-89.3283, 3.0539, 0)),
    Edge.make_line((-89.3283, 3.0539, 0), (-89.4175, 3.7054, 0)),
    Edge.make_line((-89.4175, 3.7054, 0), (-89.5066, 4.3568, 0)),
    Edge.make_line((-89.5066, 4.3568, 0), (-90.1306, 4.2981, 0)),
    Edge.make_line((-90.1306, 4.2981, 0), (-90.7813, 4.2038, 0)),
    Edge.make_line((-90.7813, 4.2038, 0), (-91.432, 4.1094, 0)),
    Edge.make_line((-91.432, 4.1094, 0), (-92.0828, 4.0151, 0)),
    Edge.make_line((-92.0828, 4.0151, 0), (-92.6238, 3.649, 0)),
    Edge.make_line((-92.6238, 3.649, 0), (-93.1579, 3.2654, 0)),
    Edge.make_line((-93.1579, 3.2654, 0), (-93.6919, 2.8818, 0)),
    Edge.make_line((-93.6919, 2.8818, 0), (-94.2259, 2.4982, 0)),
    Edge.make_line((-94.2259, 2.4982, 0), (-94.7599, 2.1146, 0)),
    Edge.make_line((-94.7599, 2.1146, 0), (-95.2939, 1.7309, 0)),
    Edge.make_line((-95.2939, 1.7309, 0), (-95.828, 1.3473, 0)),
    Edge.make_line((-95.828, 1.3473, 0), (-96.362, 0.9637, 0)),
    Edge.make_line((-96.362, 0.9637, 0), (-96.8936, 0.5767, 0)),
    Edge.make_line((-96.8936, 0.5767, 0), (-97.424, 0.1882, 0)),
    Edge.make_line((-97.424, 0.1882, 0), (-97.9545, -0.2003, 0)),
    Edge.make_line((-97.9545, -0.2003, 0), (-98.485, -0.5888, 0)),
    Edge.make_line((-98.485, -0.5888, 0), (-99.0154, -0.9773, 0)),
    Edge.make_line((-99.0154, -0.9773, 0), (-99.5459, -1.3658, 0)),
    Edge.make_line((-99.5459, -1.3658, 0), (-100.0764, -1.7544, 0)),
    Edge.make_line((-100.0764, -1.7544, 0), (-100.6068, -2.1429, 0)),
    Edge.make_line((-100.6068, -2.1429, 0), (-101.1373, -2.5314, 0)),
    Edge.make_line((-101.1373, -2.5314, 0), (-101.6678, -2.9199, 0)),
    Edge.make_line((-101.6678, -2.9199, 0), (-102.2604, -3.1927, 0)),
    Edge.make_line((-102.2604, -3.1927, 0), (-102.8772, -3.4205, 0)),
    Edge.make_line((-102.8772, -3.4205, 0), (-103.494, -3.6483, 0)),
    Edge.make_line((-103.494, -3.6483, 0), (-104.1108, -3.8761, 0)),
    Edge.make_line((-104.1108, -3.8761, 0), (-104.6649, -4.2206, 0)),
    Edge.make_line((-104.6649, -4.2206, 0), (-105.1951, -4.6094, 0)),
    Edge.make_line((-105.1951, -4.6094, 0), (-105.7254, -4.9982, 0)),
    Edge.make_line((-105.7254, -4.9982, 0), (-106.2557, -5.387, 0)),
    Edge.make_line((-106.2557, -5.387, 0), (-105.8278, -5.5304, 0)),
    Edge.make_line((-105.8278, -5.5304, 0), (-105.1759, -5.6165, 0)),
]
sk_Sketch77_loft_42_wire = Wire(sk_Sketch77_loft_42_edges)
sk_Sketch77_loft_42_face = Face(sk_Sketch77_loft_42_wire)
sk_Sketch77_loft_42_face = Plane(origin=Vector(0.0, 0.0, 10.9074), x_dir=Vector(1.0, 0.0, 0.0), z_dir=Vector(0.0, 0.0, 1.0)) * sk_Sketch77_loft_42_face
sk_Sketch78_loft_43_edges = [
    Edge.make_line((-72.7391, 51.4955, 0), (-73.3434, 51.4747, 0)),
    Edge.make_line((-73.3434, 51.4747, 0), (-73.9476, 51.4539, 0)),
    Edge.make_line((-73.9476, 51.4539, 0), (-74.5519, 51.4331, 0)),
    Edge.make_line((-74.5519, 51.4331, 0), (-75.1559, 51.4059, 0)),
    Edge.make_line((-75.1559, 51.4059, 0), (-75.7593, 51.3684, 0)),
    Edge.make_line((-75.7593, 51.3684, 0), (-76.3628, 51.3309, 0)),
    Edge.make_line((-76.3628, 51.3309, 0), (-76.9662, 51.2934, 0)),
    Edge.make_line((-76.9662, 51.2934, 0), (-77.5697, 51.2559, 0)),
    Edge.make_line((-77.5697, 51.2559, 0), (-78.1732, 51.2184, 0)),
    Edge.make_line((-78.1732, 51.2184, 0), (-78.7766, 51.1809, 0)),
    Edge.make_line((-78.7766, 51.1809, 0), (-79.3801, 51.1434, 0)),
    Edge.make_line((-79.3801, 51.1434, 0), (-79.9732, 51.2426, 0)),
    Edge.make_line((-79.9732, 51.2426, 0), (-80.5641, 51.3705, 0)),
    Edge.make_line((-80.5641, 51.3705, 0), (-81.155, 51.4985, 0)),
    Edge.make_line((-81.155, 51.4985, 0), (-81.746, 51.6265, 0)),
    Edge.make_line((-81.746, 51.6265, 0), (-82.3369, 51.7544, 0)),
    Edge.make_line((-82.3369, 51.7544, 0), (-82.9278, 51.8824, 0)),
    Edge.make_line((-82.9278, 51.8824, 0), (-83.3791, 51.5192, 0)),
    Edge.make_line((-83.3791, 51.5192, 0), (-83.8144, 51.0996, 0)),
    Edge.make_line((-83.8144, 51.0996, 0), (-84.2497, 50.6799, 0)),
    Edge.make_line((-84.2497, 50.6799, 0), (-84.685, 50.2603, 0)),
    Edge.make_line((-84.685, 50.2603, 0), (-85.1203, 49.8407, 0)),
    Edge.make_line((-85.1203, 49.8407, 0), (-85.5556, 49.421, 0)),
    Edge.make_line((-85.5556, 49.421, 0), (-85.0622, 49.2878, 0)),
    Edge.make_line((-85.0622, 49.2878, 0), (-84.4661, 49.1862, 0)),
    Edge.make_line((-84.4661, 49.1862, 0), (-83.8701, 49.0846, 0)),
    Edge.make_line((-83.8701, 49.0846, 0), (-83.2741, 48.9829, 0)),
    Edge.make_line((-83.2741, 48.9829, 0), (-82.6781, 48.8813, 0)),
    Edge.make_line((-82.6781, 48.8813, 0), (-82.082, 48.7797, 0)),
    Edge.make_line((-82.082, 48.7797, 0), (-81.486, 48.6781, 0)),
    Edge.make_line((-81.486, 48.6781, 0), (-80.8865, 48.6174, 0)),
    Edge.make_line((-80.8865, 48.6174, 0), (-80.2818, 48.6149, 0)),
    Edge.make_line((-80.2818, 48.6149, 0), (-79.6772, 48.6124, 0)),
    Edge.make_line((-79.6772, 48.6124, 0), (-79.0726, 48.61, 0)),
    Edge.make_line((-79.0726, 48.61, 0), (-78.468, 48.6075, 0)),
    Edge.make_line((-78.468, 48.6075, 0), (-77.8634, 48.605, 0)),
    Edge.make_line((-77.8634, 48.605, 0), (-77.2587, 48.6026, 0)),
    Edge.make_line((-77.2587, 48.6026, 0), (-76.6541, 48.6001, 0)),
    Edge.make_line((-76.6541, 48.6001, 0), (-76.0495, 48.5976, 0)),
    Edge.make_line((-76.0495, 48.5976, 0), (-75.4449, 48.5952, 0)),
    Edge.make_line((-75.4449, 48.5952, 0), (-74.8403, 48.5927, 0)),
    Edge.make_line((-74.8403, 48.5927, 0), (-74.2356, 48.5902, 0)),
    Edge.make_line((-74.2356, 48.5902, 0), (-73.631, 48.5878, 0)),
    Edge.make_line((-73.631, 48.5878, 0), (-73.0264, 48.5853, 0)),
    Edge.make_line((-73.0264, 48.5853, 0), (-72.4332, 48.4729, 0)),
    Edge.make_line((-72.4332, 48.4729, 0), (-71.841, 48.351, 0)),
    Edge.make_line((-71.841, 48.351, 0), (-71.2488, 48.2292, 0)),
    Edge.make_line((-71.2488, 48.2292, 0), (-70.6565, 48.1074, 0)),
    Edge.make_line((-70.6565, 48.1074, 0), (-70.0643, 47.9855, 0)),
    Edge.make_line((-70.0643, 47.9855, 0), (-69.4721, 47.8637, 0)),
    Edge.make_line((-69.4721, 47.8637, 0), (-69.09, 48.0979, 0)),
    Edge.make_line((-69.09, 48.0979, 0), (-68.9102, 48.6752, 0)),
    Edge.make_line((-68.9102, 48.6752, 0), (-68.7305, 49.2525, 0)),
    Edge.make_line((-68.7305, 49.2525, 0), (-68.5507, 49.8298, 0)),
    Edge.make_line((-68.5507, 49.8298, 0), (-68.371, 50.4071, 0)),
    Edge.make_line((-68.371, 50.4071, 0), (-68.1912, 50.9844, 0)),
    Edge.make_line((-68.1912, 50.9844, 0), (-68.516, 51.2155, 0)),
    Edge.make_line((-68.516, 51.2155, 0), (-69.1193, 51.2555, 0)),
    Edge.make_line((-69.1193, 51.2555, 0), (-69.7226, 51.2955, 0)),
    Edge.make_line((-69.7226, 51.2955, 0), (-70.3259, 51.3355, 0)),
    Edge.make_line((-70.3259, 51.3355, 0), (-70.9292, 51.3755, 0)),
    Edge.make_line((-70.9292, 51.3755, 0), (-71.5325, 51.4155, 0)),
    Edge.make_line((-71.5325, 51.4155, 0), (-72.1358, 51.4555, 0)),
    Edge.make_line((-72.1358, 51.4555, 0), (-72.7391, 51.4955, 0)),
]
sk_Sketch78_loft_43_wire = Wire(sk_Sketch78_loft_43_edges)
sk_Sketch78_loft_43_face = Face(sk_Sketch78_loft_43_wire)
sk_Sketch78_loft_43_face = Plane(origin=Vector(-8.494, -5.0626, 36.5935), x_dir=Vector(0.829254, 0.494255, 0.260863), z_dir=Vector(-0.22408, -0.133557, 0.965376)) * sk_Sketch78_loft_43_face
sk_Sketch81_loft_44_edges = [
    Edge.make_line((-51.1369, -79.4853, 0), (-51.1602, -80.0801, 0)),
    Edge.make_line((-51.1602, -80.0801, 0), (-51.1834, -80.6748, 0)),
    Edge.make_line((-51.1834, -80.6748, 0), (-51.2067, -81.2696, 0)),
    Edge.make_line((-51.2067, -81.2696, 0), (-51.0776, -81.8485, 0)),
    Edge.make_line((-51.0776, -81.8485, 0), (-50.9328, -82.4258, 0)),
    Edge.make_line((-50.9328, -82.4258, 0), (-50.788, -83.0032, 0)),
    Edge.make_line((-50.788, -83.0032, 0), (-50.6432, -83.5805, 0)),
    Edge.make_line((-50.6432, -83.5805, 0), (-50.4983, -84.1578, 0)),
    Edge.make_line((-50.4983, -84.1578, 0), (-50.2613, -84.684, 0)),
    Edge.make_line((-50.2613, -84.684, 0), (-49.8482, -85.1125, 0)),
    Edge.make_line((-49.8482, -85.1125, 0), (-49.4351, -85.541, 0)),
    Edge.make_line((-49.4351, -85.541, 0), (-49.2928, -85.0914, 0)),
    Edge.make_line((-49.2928, -85.0914, 0), (-49.1928, -84.5047, 0)),
    Edge.make_line((-49.1928, -84.5047, 0), (-49.0928, -83.9179, 0)),
    Edge.make_line((-49.0928, -83.9179, 0), (-48.9928, -83.3312, 0)),
    Edge.make_line((-48.9928, -83.3312, 0), (-48.8928, -82.7444, 0)),
    Edge.make_line((-48.8928, -82.7444, 0), (-48.7928, -82.1577, 0)),
    Edge.make_line((-48.7928, -82.1577, 0), (-48.6928, -81.5709, 0)),
    Edge.make_line((-48.6928, -81.5709, 0), (-48.6177, -80.982, 0)),
    Edge.make_line((-48.6177, -80.982, 0), (-48.6153, -80.3868, 0)),
    Edge.make_line((-48.6153, -80.3868, 0), (-48.6128, -79.7916, 0)),
    Edge.make_line((-48.6128, -79.7916, 0), (-48.6104, -79.1964, 0)),
    Edge.make_line((-48.6104, -79.1964, 0), (-48.608, -78.6012, 0)),
    Edge.make_line((-48.608, -78.6012, 0), (-48.6056, -78.006, 0)),
    Edge.make_line((-48.6056, -78.006, 0), (-48.6031, -77.4108, 0)),
    Edge.make_line((-48.6031, -77.4108, 0), (-48.6007, -76.8156, 0)),
    Edge.make_line((-48.6007, -76.8156, 0), (-48.5983, -76.2204, 0)),
    Edge.make_line((-48.5983, -76.2204, 0), (-48.5959, -75.6252, 0)),
    Edge.make_line((-48.5959, -75.6252, 0), (-48.5935, -75.03, 0)),
    Edge.make_line((-48.5935, -75.03, 0), (-48.591, -74.4348, 0)),
    Edge.make_line((-48.591, -74.4348, 0), (-48.5886, -73.8396, 0)),
    Edge.make_line((-48.5886, -73.8396, 0), (-48.5862, -73.2444, 0)),
    Edge.make_line((-48.5862, -73.2444, 0), (-48.5187, -72.656, 0)),
    Edge.make_line((-48.5187, -72.656, 0), (-48.3987, -72.073, 0)),
    Edge.make_line((-48.3987, -72.073, 0), (-48.2788, -71.49, 0)),
    Edge.make_line((-48.2788, -71.49, 0), (-48.1589, -70.907, 0)),
    Edge.make_line((-48.1589, -70.907, 0), (-48.0389, -70.324, 0)),
    Edge.make_line((-48.0389, -70.324, 0), (-47.919, -69.741, 0)),
    Edge.make_line((-47.919, -69.741, 0), (-47.8268, -69.1744, 0)),
    Edge.make_line((-47.8268, -69.1744, 0), (-48.3951, -68.9974, 0)),
    Edge.make_line((-48.3951, -68.9974, 0), (-48.9634, -68.8205, 0)),
    Edge.make_line((-48.9634, -68.8205, 0), (-49.5317, -68.6435, 0)),
    Edge.make_line((-49.5317, -68.6435, 0), (-50.1, -68.4666, 0)),
    Edge.make_line((-50.1, -68.4666, 0), (-50.6683, -68.2896, 0)),
    Edge.make_line((-50.6683, -68.2896, 0), (-51.1895, -68.1765, 0)),
    Edge.make_line((-51.1895, -68.1765, 0), (-51.1867, -68.7717, 0)),
    Edge.make_line((-51.1867, -68.7717, 0), (-51.1839, -69.3669, 0)),
    Edge.make_line((-51.1839, -69.3669, 0), (-51.1812, -69.9621, 0)),
    Edge.make_line((-51.1812, -69.9621, 0), (-51.1784, -70.5573, 0)),
    Edge.make_line((-51.1784, -70.5573, 0), (-51.1756, -71.1525, 0)),
    Edge.make_line((-51.1756, -71.1525, 0), (-51.1729, -71.7477, 0)),
    Edge.make_line((-51.1729, -71.7477, 0), (-51.1701, -72.3429, 0)),
    Edge.make_line((-51.1701, -72.3429, 0), (-51.1673, -72.9381, 0)),
    Edge.make_line((-51.1673, -72.9381, 0), (-51.1646, -73.5333, 0)),
    Edge.make_line((-51.1646, -73.5333, 0), (-51.1618, -74.1285, 0)),
    Edge.make_line((-51.1618, -74.1285, 0), (-51.159, -74.7237, 0)),
    Edge.make_line((-51.159, -74.7237, 0), (-51.1563, -75.3189, 0)),
    Edge.make_line((-51.1563, -75.3189, 0), (-51.1535, -75.9141, 0)),
    Edge.make_line((-51.1535, -75.9141, 0), (-51.1507, -76.5093, 0)),
    Edge.make_line((-51.1507, -76.5093, 0), (-51.148, -77.1045, 0)),
    Edge.make_line((-51.148, -77.1045, 0), (-51.1452, -77.6997, 0)),
    Edge.make_line((-51.1452, -77.6997, 0), (-51.1424, -78.2949, 0)),
    Edge.make_line((-51.1424, -78.2949, 0), (-51.1397, -78.8901, 0)),
    Edge.make_line((-51.1397, -78.8901, 0), (-51.1369, -79.4853, 0)),
]
sk_Sketch81_loft_44_wire = Wire(sk_Sketch81_loft_44_edges)
sk_Sketch81_loft_44_face = Face(sk_Sketch81_loft_44_wire)
sk_Sketch81_loft_44_face = Plane(origin=Vector(-8.494, -5.0626, 36.5935), x_dir=Vector(0.511982, -0.858996, 0.0), z_dir=Vector(-0.22408, -0.133557, 0.965376)) * sk_Sketch81_loft_44_face
sk_Sketch79_loft_45_edges = [
    Edge.make_line((-66.8278, -58.3, 0), (-66.175, -58.3206, 0)),
    Edge.make_line((-66.175, -58.3206, 0), (-65.5222, -58.3412, 0)),
    Edge.make_line((-65.5222, -58.3412, 0), (-64.8693, -58.3618, 0)),
    Edge.make_line((-64.8693, -58.3618, 0), (-64.2165, -58.3823, 0)),
    Edge.make_line((-64.2165, -58.3823, 0), (-64.0002, -57.9158, 0)),
    Edge.make_line((-64.0002, -57.9158, 0), (-63.9494, -57.2647, 0)),
    Edge.make_line((-63.9494, -57.2647, 0), (-63.8985, -56.6135, 0)),
    Edge.make_line((-63.8985, -56.6135, 0), (-63.8476, -55.9623, 0)),
    Edge.make_line((-63.8476, -55.9623, 0), (-63.7968, -55.3112, 0)),
    Edge.make_line((-63.7968, -55.3112, 0), (-63.7459, -54.66, 0)),
    Edge.make_line((-63.7459, -54.66, 0), (-63.695, -54.0089, 0)),
    Edge.make_line((-63.695, -54.0089, 0), (-63.6442, -53.3577, 0)),
    Edge.make_line((-63.6442, -53.3577, 0), (-63.5933, -52.7066, 0)),
    Edge.make_line((-63.5933, -52.7066, 0), (-63.5408, -52.0556, 0)),
    Edge.make_line((-63.5408, -52.0556, 0), (-63.4706, -51.4062, 0)),
    Edge.make_line((-63.4706, -51.4062, 0), (-63.4003, -50.7568, 0)),
    Edge.make_line((-63.4003, -50.7568, 0), (-63.3301, -50.1075, 0)),
    Edge.make_line((-63.3301, -50.1075, 0), (-63.2599, -49.4581, 0)),
    Edge.make_line((-63.2599, -49.4581, 0), (-63.1897, -48.8088, 0)),
    Edge.make_line((-63.1897, -48.8088, 0), (-63.1194, -48.1594, 0)),
    Edge.make_line((-63.1194, -48.1594, 0), (-63.0348, -47.5127, 0)),
    Edge.make_line((-63.0348, -47.5127, 0), (-62.8685, -46.8811, 0)),
    Edge.make_line((-62.8685, -46.8811, 0), (-62.7023, -46.2495, 0)),
    Edge.make_line((-62.7023, -46.2495, 0), (-62.5361, -45.6178, 0)),
    Edge.make_line((-62.5361, -45.6178, 0), (-62.3699, -44.9862, 0)),
    Edge.make_line((-62.3699, -44.9862, 0), (-62.2037, -44.3545, 0)),
    Edge.make_line((-62.2037, -44.3545, 0), (-62.0375, -43.7229, 0)),
    Edge.make_line((-62.0375, -43.7229, 0), (-61.8713, -43.0913, 0)),
    Edge.make_line((-61.8713, -43.0913, 0), (-61.7051, -42.4596, 0)),
    Edge.make_line((-61.7051, -42.4596, 0), (-61.981, -41.8738, 0)),
    Edge.make_line((-61.981, -41.8738, 0), (-62.2485, -41.2803, 0)),
    Edge.make_line((-62.2485, -41.2803, 0), (-62.8554, -41.1271, 0)),
    Edge.make_line((-62.8554, -41.1271, 0), (-63.4986, -41.0131, 0)),
    Edge.make_line((-63.4986, -41.0131, 0), (-64.1417, -40.8991, 0)),
    Edge.make_line((-64.1417, -40.8991, 0), (-64.7848, -40.7851, 0)),
    Edge.make_line((-64.7848, -40.7851, 0), (-65.4279, -40.6711, 0)),
    Edge.make_line((-65.4279, -40.6711, 0), (-65.9665, -40.6892, 0)),
    Edge.make_line((-65.9665, -40.6892, 0), (-66.0028, -41.3413, 0)),
    Edge.make_line((-66.0028, -41.3413, 0), (-66.0392, -41.9934, 0)),
    Edge.make_line((-66.0392, -41.9934, 0), (-66.0756, -42.6456, 0)),
    Edge.make_line((-66.0756, -42.6456, 0), (-66.1119, -43.2977, 0)),
    Edge.make_line((-66.1119, -43.2977, 0), (-66.1483, -43.9498, 0)),
    Edge.make_line((-66.1483, -43.9498, 0), (-66.1846, -44.602, 0)),
    Edge.make_line((-66.1846, -44.602, 0), (-66.221, -45.2541, 0)),
    Edge.make_line((-66.221, -45.2541, 0), (-66.2574, -45.9062, 0)),
    Edge.make_line((-66.2574, -45.9062, 0), (-66.2937, -46.5583, 0)),
    Edge.make_line((-66.2937, -46.5583, 0), (-66.3301, -47.2105, 0)),
    Edge.make_line((-66.3301, -47.2105, 0), (-66.3664, -47.8626, 0)),
    Edge.make_line((-66.3664, -47.8626, 0), (-66.4028, -48.5147, 0)),
    Edge.make_line((-66.4028, -48.5147, 0), (-66.4392, -49.1668, 0)),
    Edge.make_line((-66.4392, -49.1668, 0), (-66.4755, -49.819, 0)),
    Edge.make_line((-66.4755, -49.819, 0), (-66.5119, -50.4711, 0)),
    Edge.make_line((-66.5119, -50.4711, 0), (-66.5483, -51.1232, 0)),
    Edge.make_line((-66.5483, -51.1232, 0), (-66.5846, -51.7754, 0)),
    Edge.make_line((-66.5846, -51.7754, 0), (-66.621, -52.4275, 0)),
    Edge.make_line((-66.621, -52.4275, 0), (-66.6573, -53.0796, 0)),
    Edge.make_line((-66.6573, -53.0796, 0), (-66.6937, -53.7317, 0)),
    Edge.make_line((-66.6937, -53.7317, 0), (-66.7301, -54.3839, 0)),
    Edge.make_line((-66.7301, -54.3839, 0), (-66.7664, -55.036, 0)),
    Edge.make_line((-66.7664, -55.036, 0), (-66.8028, -55.6881, 0)),
    Edge.make_line((-66.8028, -55.6881, 0), (-66.8278, -56.3406, 0)),
    Edge.make_line((-66.8278, -56.3406, 0), (-66.8278, -56.9937, 0)),
    Edge.make_line((-66.8278, -56.9937, 0), (-66.8278, -57.6469, 0)),
    Edge.make_line((-66.8278, -57.6469, 0), (-66.8278, -58.3, 0)),
]
sk_Sketch79_loft_45_wire = Wire(sk_Sketch79_loft_45_edges)
sk_Sketch79_loft_45_face = Face(sk_Sketch79_loft_45_wire)
sk_Sketch79_loft_45_face = Plane(origin=Vector(-24.7305, -20.3355, 51.0393), x_dir=Vector(0.635133, -0.772403, 0.0), z_dir=Vector(-0.410461, -0.337515, 0.847116)) * sk_Sketch79_loft_45_face
_work_plane_46 = Plane(
    origin=Vector(-91.3238, 49.7362, 0.0),
    x_dir=Vector(0.478284, 0.878205, 0.0),
    z_dir=Vector(0.878205, -0.478284, -0.0),
)
with BuildSketch(_work_plane_46) as sk_Sketch84_46:
    with BuildLine():
        Line((-20.1326, 26.3484), (-20.315, 20.9074))
        Line((-20.315, 20.9074), (-23.7244, 20.9074))
        RadiusArc((-23.7244, 20.9074), (-20.1326, 26.3484), -15.0296)
    _edges_sk_Sketch84_46 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch84_46 = Wire.combine(_edges_sk_Sketch84_46)[0]
_w_sk_Sketch84_46 = _w_sk_Sketch84_46.moved(_work_plane_46.location)
_mf_sk_Sketch84_46 = BRepBuilderAPI_MakeFace(_work_plane_46.wrapped, _w_sk_Sketch84_46.wrapped, True)
_face_sk_Sketch84_46 = Face(_mf_sk_Sketch84_46.Face())
sk_Sketch88_loft_47_edges = [
    Edge.make_line((-85.9856, -25.3297, 0), (-86.2002, -26.43, 0)),
    Edge.make_line((-86.2002, -26.43, 0), (-86.4149, -27.5303, 0)),
    Edge.make_line((-86.4149, -27.5303, 0), (-86.6295, -28.6306, 0)),
    Edge.make_line((-86.6295, -28.6306, 0), (-86.8442, -29.7309, 0)),
    Edge.make_line((-86.8442, -29.7309, 0), (-87.0588, -30.8312, 0)),
    Edge.make_line((-87.0588, -30.8312, 0), (-87.2734, -31.9315, 0)),
    Edge.make_line((-87.2734, -31.9315, 0), (-87.4881, -33.0318, 0)),
    Edge.make_line((-87.4881, -33.0318, 0), (-87.7027, -34.1321, 0)),
    Edge.make_line((-87.7027, -34.1321, 0), (-87.9174, -35.2324, 0)),
    Edge.make_line((-87.9174, -35.2324, 0), (-88.132, -36.3327, 0)),
    Edge.make_line((-88.132, -36.3327, 0), (-88.3467, -37.433, 0)),
    Edge.make_line((-88.3467, -37.433, 0), (-88.5613, -38.5333, 0)),
    Edge.make_line((-88.5613, -38.5333, 0), (-88.7759, -39.6336, 0)),
    Edge.make_line((-88.7759, -39.6336, 0), (-88.9906, -40.7339, 0)),
    Edge.make_line((-88.9906, -40.7339, 0), (-89.2052, -41.8342, 0)),
    Edge.make_line((-89.2052, -41.8342, 0), (-89.4199, -42.9345, 0)),
    Edge.make_line((-89.4199, -42.9345, 0), (-89.6345, -44.0348, 0)),
    Edge.make_line((-89.6345, -44.0348, 0), (-89.8491, -45.1351, 0)),
    Edge.make_line((-89.8491, -45.1351, 0), (-90.0638, -46.2354, 0)),
    Edge.make_line((-90.0638, -46.2354, 0), (-90.2784, -47.3357, 0)),
    Edge.make_line((-90.2784, -47.3357, 0), (-90.4931, -48.436, 0)),
    Edge.make_line((-90.4931, -48.436, 0), (-90.7077, -49.5363, 0)),
    Edge.make_line((-90.7077, -49.5363, 0), (-90.9224, -50.6366, 0)),
    Edge.make_line((-90.9224, -50.6366, 0), (-91.137, -51.7369, 0)),
    Edge.make_line((-91.137, -51.7369, 0), (-91.3516, -52.8372, 0)),
    Edge.make_line((-91.3516, -52.8372, 0), (-91.5663, -53.9375, 0)),
    Edge.make_line((-91.5663, -53.9375, 0), (-90.5465, -54.3533, 0)),
    Edge.make_line((-90.5465, -54.3533, 0), (-89.4995, -54.754, 0)),
    Edge.make_line((-89.4995, -54.754, 0), (-89.2386, -53.7372, 0)),
    Edge.make_line((-89.2386, -53.7372, 0), (-89.0239, -52.6369, 0)),
    Edge.make_line((-89.0239, -52.6369, 0), (-88.7995, -51.5386, 0)),
    Edge.make_line((-88.7995, -51.5386, 0), (-88.5599, -50.4434, 0)),
    Edge.make_line((-88.5599, -50.4434, 0), (-88.3203, -49.3483, 0)),
    Edge.make_line((-88.3203, -49.3483, 0), (-88.0806, -48.2532, 0)),
    Edge.make_line((-88.0806, -48.2532, 0), (-87.841, -47.158, 0)),
    Edge.make_line((-87.841, -47.158, 0), (-87.6014, -46.0629, 0)),
    Edge.make_line((-87.6014, -46.0629, 0), (-87.3618, -44.9678, 0)),
    Edge.make_line((-87.3618, -44.9678, 0), (-87.1222, -43.8726, 0)),
    Edge.make_line((-87.1222, -43.8726, 0), (-86.8826, -42.7775, 0)),
    Edge.make_line((-86.8826, -42.7775, 0), (-86.643, -41.6824, 0)),
    Edge.make_line((-86.643, -41.6824, 0), (-86.4034, -40.5872, 0)),
    Edge.make_line((-86.4034, -40.5872, 0), (-86.1637, -39.4921, 0)),
    Edge.make_line((-86.1637, -39.4921, 0), (-85.9241, -38.397, 0)),
    Edge.make_line((-85.9241, -38.397, 0), (-85.6845, -37.3018, 0)),
    Edge.make_line((-85.6845, -37.3018, 0), (-85.4449, -36.2067, 0)),
    Edge.make_line((-85.4449, -36.2067, 0), (-85.2053, -35.1116, 0)),
    Edge.make_line((-85.2053, -35.1116, 0), (-84.9657, -34.0164, 0)),
    Edge.make_line((-84.9657, -34.0164, 0), (-84.7261, -32.9213, 0)),
    Edge.make_line((-84.7261, -32.9213, 0), (-84.4865, -31.8262, 0)),
    Edge.make_line((-84.4865, -31.8262, 0), (-84.2468, -30.731, 0)),
    Edge.make_line((-84.2468, -30.731, 0), (-83.9614, -29.6475, 0)),
    Edge.make_line((-83.9614, -29.6475, 0), (-83.6497, -28.5707, 0)),
    Edge.make_line((-83.6497, -28.5707, 0), (-83.338, -27.4939, 0)),
    Edge.make_line((-83.338, -27.4939, 0), (-83.0262, -26.417, 0)),
    Edge.make_line((-83.0262, -26.417, 0), (-82.7145, -25.3402, 0)),
    Edge.make_line((-82.7145, -25.3402, 0), (-82.4028, -24.2634, 0)),
    Edge.make_line((-82.4028, -24.2634, 0), (-82.0911, -23.1866, 0)),
    Edge.make_line((-82.0911, -23.1866, 0), (-81.8813, -22.1664, 0)),
    Edge.make_line((-81.8813, -22.1664, 0), (-82.9605, -21.8629, 0)),
    Edge.make_line((-82.9605, -21.8629, 0), (-84.0397, -21.5594, 0)),
    Edge.make_line((-84.0397, -21.5594, 0), (-84.6491, -22.2436, 0)),
    Edge.make_line((-84.6491, -22.2436, 0), (-85.0946, -23.2723, 0)),
    Edge.make_line((-85.0946, -23.2723, 0), (-85.5401, -24.301, 0)),
    Edge.make_line((-85.5401, -24.301, 0), (-85.9856, -25.3297, 0)),
]
sk_Sketch88_loft_47_wire = Wire(sk_Sketch88_loft_47_edges)
sk_Sketch88_loft_47_face = Face(sk_Sketch88_loft_47_wire)
sk_Sketch88_loft_47_face = Plane(origin=Vector(-18.5737, -18.4741, 47.5662), x_dir=Vector(0.705203, -0.709006, -0.0), z_dir=Vector(-0.342038, -0.340204, 0.87594)) * sk_Sketch88_loft_47_face
sk_Sketch87_loft_48_edges = [
    Edge.make_line((84.2404, 35.6633, 0), (84.3323, 34.6998, 0)),
    Edge.make_line((84.3323, 34.6998, 0), (84.4501, 33.7385, 0)),
    Edge.make_line((84.4501, 33.7385, 0), (84.568, 32.7773, 0)),
    Edge.make_line((84.568, 32.7773, 0), (84.6859, 31.816, 0)),
    Edge.make_line((84.6859, 31.816, 0), (84.8685, 30.8695, 0)),
    Edge.make_line((84.8685, 30.8695, 0), (85.1757, 29.951, 0)),
    Edge.make_line((85.1757, 29.951, 0), (85.4828, 29.0326, 0)),
    Edge.make_line((85.4828, 29.0326, 0), (85.7894, 28.9387, 0)),
    Edge.make_line((85.7894, 28.9387, 0), (86.3295, 28.7516, 0)),
    Edge.make_line((86.3295, 28.7516, 0), (87.0705, 29.3751, 0)),
    Edge.make_line((87.0705, 29.3751, 0), (87.8108, 29.9988, 0)),
    Edge.make_line((87.8108, 29.9988, 0), (87.2615, 30.7257, 0)),
    Edge.make_line((87.2615, 30.7257, 0), (86.9352, 31.541, 0)),
    Edge.make_line((86.9352, 31.541, 0), (86.6281, 32.4594, 0)),
    Edge.make_line((86.6281, 32.4594, 0), (86.3209, 33.3778, 0)),
    Edge.make_line((86.3209, 33.3778, 0), (86.1907, 34.3262, 0)),
    Edge.make_line((86.1907, 34.3262, 0), (86.178, 35.2945, 0)),
    Edge.make_line((86.178, 35.2945, 0), (86.1535, 36.262, 0)),
    Edge.make_line((86.1535, 36.262, 0), (86.1688, 37.2304, 0)),
    Edge.make_line((86.1688, 37.2304, 0), (86.184, 38.1987, 0)),
    Edge.make_line((86.184, 38.1987, 0), (86.1993, 39.167, 0)),
    Edge.make_line((86.1993, 39.167, 0), (86.2145, 40.1353, 0)),
    Edge.make_line((86.2145, 40.1353, 0), (86.2298, 41.1036, 0)),
    Edge.make_line((86.2298, 41.1036, 0), (86.2451, 42.0719, 0)),
    Edge.make_line((86.2451, 42.0719, 0), (86.2548, 43.0403, 0)),
    Edge.make_line((86.2548, 43.0403, 0), (86.2548, 44.0087, 0)),
    Edge.make_line((86.2548, 44.0087, 0), (86.2548, 44.9772, 0)),
    Edge.make_line((86.2548, 44.9772, 0), (86.2548, 45.9456, 0)),
    Edge.make_line((86.2548, 45.9456, 0), (86.2548, 46.914, 0)),
    Edge.make_line((86.2548, 46.914, 0), (86.2548, 47.8825, 0)),
    Edge.make_line((86.2548, 47.8825, 0), (86.2548, 48.8509, 0)),
    Edge.make_line((86.2548, 48.8509, 0), (86.2548, 49.8193, 0)),
    Edge.make_line((86.2548, 49.8193, 0), (86.2515, 50.7876, 0)),
    Edge.make_line((86.2515, 50.7876, 0), (86.1535, 51.7511, 0)),
    Edge.make_line((86.1535, 51.7511, 0), (86.0555, 52.7145, 0)),
    Edge.make_line((86.0555, 52.7145, 0), (85.9575, 53.678, 0)),
    Edge.make_line((85.9575, 53.678, 0), (85.8595, 54.6415, 0)),
    Edge.make_line((85.8595, 54.6415, 0), (85.7615, 55.6049, 0)),
    Edge.make_line((85.7615, 55.6049, 0), (85.6635, 56.5684, 0)),
    Edge.make_line((85.6635, 56.5684, 0), (85.2036, 57.1313, 0)),
    Edge.make_line((85.2036, 57.1313, 0), (84.2351, 57.1313, 0)),
    Edge.make_line((84.2351, 57.1313, 0), (83.5683, 56.8789, 0)),
    Edge.make_line((83.5683, 56.8789, 0), (83.7389, 55.9256, 0)),
    Edge.make_line((83.7389, 55.9256, 0), (83.9096, 54.9724, 0)),
    Edge.make_line((83.9096, 54.9724, 0), (84.063, 54.017, 0)),
    Edge.make_line((84.063, 54.017, 0), (84.1227, 53.0504, 0)),
    Edge.make_line((84.1227, 53.0504, 0), (84.1823, 52.0838, 0)),
    Edge.make_line((84.1823, 52.0838, 0), (84.242, 51.1172, 0)),
    Edge.make_line((84.242, 51.1172, 0), (84.2548, 50.1898, 0)),
    Edge.make_line((84.2548, 50.1898, 0), (84.2548, 49.2214, 0)),
    Edge.make_line((84.2548, 49.2214, 0), (84.2548, 48.2529, 0)),
    Edge.make_line((84.2548, 48.2529, 0), (84.2548, 47.2845, 0)),
    Edge.make_line((84.2548, 47.2845, 0), (84.2548, 46.3161, 0)),
    Edge.make_line((84.2548, 46.3161, 0), (84.2548, 45.3476, 0)),
    Edge.make_line((84.2548, 45.3476, 0), (84.2548, 44.3792, 0)),
    Edge.make_line((84.2548, 44.3792, 0), (84.2548, 43.4108, 0)),
    Edge.make_line((84.2548, 43.4108, 0), (84.2543, 42.4423, 0)),
    Edge.make_line((84.2543, 42.4423, 0), (84.2523, 41.4739, 0)),
    Edge.make_line((84.2523, 41.4739, 0), (84.2503, 40.5055, 0)),
    Edge.make_line((84.2503, 40.5055, 0), (84.2483, 39.537, 0)),
    Edge.make_line((84.2483, 39.537, 0), (84.2464, 38.5686, 0)),
    Edge.make_line((84.2464, 38.5686, 0), (84.2444, 37.6002, 0)),
    Edge.make_line((84.2444, 37.6002, 0), (84.2424, 36.6317, 0)),
    Edge.make_line((84.2424, 36.6317, 0), (84.2404, 35.6633, 0)),
]
sk_Sketch87_loft_48_wire = Wire(sk_Sketch87_loft_48_edges)
sk_Sketch87_loft_48_face = Face(sk_Sketch87_loft_48_wire)
sk_Sketch87_loft_48_face = Plane(origin=Vector(-28.9119, -22.4594, 43.6306), x_dir=Vector(-0.613469, 0.789719, 1e-06), z_dir=Vector(-0.507621, -0.394331, 0.766044)) * sk_Sketch87_loft_48_face
sk_Sketch83_loft_49_edges = [
    Edge.make_line((25.0763, 16.6521, 0), (24.9473, 16.0702, 0)),
    Edge.make_line((24.9473, 16.0702, 0), (24.8183, 15.4883, 0)),
    Edge.make_line((24.8183, 15.4883, 0), (24.6893, 14.9064, 0)),
    Edge.make_line((24.6893, 14.9064, 0), (24.5603, 14.3245, 0)),
    Edge.make_line((24.5603, 14.3245, 0), (24.4313, 13.7426, 0)),
    Edge.make_line((24.4313, 13.7426, 0), (24.3023, 13.1607, 0)),
    Edge.make_line((24.3023, 13.1607, 0), (24.1733, 12.5789, 0)),
    Edge.make_line((24.1733, 12.5789, 0), (24.0443, 11.997, 0)),
    Edge.make_line((24.0443, 11.997, 0), (23.9153, 11.4151, 0)),
    Edge.make_line((23.9153, 11.4151, 0), (23.7863, 10.8332, 0)),
    Edge.make_line((23.7863, 10.8332, 0), (23.6573, 10.2513, 0)),
    Edge.make_line((23.6573, 10.2513, 0), (23.5283, 9.6694, 0)),
    Edge.make_line((23.5283, 9.6694, 0), (23.4392, 9.1171, 0)),
    Edge.make_line((23.4392, 9.1171, 0), (24.0334, 9.0708, 0)),
    Edge.make_line((24.0334, 9.0708, 0), (24.6276, 9.0246, 0)),
    Edge.make_line((24.6276, 9.0246, 0), (25.2219, 8.9784, 0)),
    Edge.make_line((25.2219, 8.9784, 0), (25.8161, 8.9322, 0)),
    Edge.make_line((25.8161, 8.9322, 0), (26.4103, 8.886, 0)),
    Edge.make_line((26.4103, 8.886, 0), (26.8311, 9.0774, 0)),
    Edge.make_line((26.8311, 9.0774, 0), (26.9682, 9.6575, 0)),
    Edge.make_line((26.9682, 9.6575, 0), (27.1054, 10.2375, 0)),
    Edge.make_line((27.1054, 10.2375, 0), (27.2426, 10.8175, 0)),
    Edge.make_line((27.2426, 10.8175, 0), (27.3797, 11.3975, 0)),
    Edge.make_line((27.3797, 11.3975, 0), (27.5169, 11.9776, 0)),
    Edge.make_line((27.5169, 11.9776, 0), (27.6541, 12.5576, 0)),
    Edge.make_line((27.6541, 12.5576, 0), (27.7913, 13.1376, 0)),
    Edge.make_line((27.7913, 13.1376, 0), (27.9284, 13.7176, 0)),
    Edge.make_line((27.9284, 13.7176, 0), (28.0656, 14.2976, 0)),
    Edge.make_line((28.0656, 14.2976, 0), (28.2028, 14.8777, 0)),
    Edge.make_line((28.2028, 14.8777, 0), (28.3399, 15.4577, 0)),
    Edge.make_line((28.3399, 15.4577, 0), (28.4771, 16.0377, 0)),
    Edge.make_line((28.4771, 16.0377, 0), (28.6143, 16.6177, 0)),
    Edge.make_line((28.6143, 16.6177, 0), (28.8561, 17.1618, 0)),
    Edge.make_line((28.8561, 17.1618, 0), (29.1046, 17.7035, 0)),
    Edge.make_line((29.1046, 17.7035, 0), (29.3531, 18.2453, 0)),
    Edge.make_line((29.3531, 18.2453, 0), (29.6016, 18.787, 0)),
    Edge.make_line((29.6016, 18.787, 0), (29.85, 19.3288, 0)),
    Edge.make_line((29.85, 19.3288, 0), (30.0985, 19.8705, 0)),
    Edge.make_line((30.0985, 19.8705, 0), (30.347, 20.4123, 0)),
    Edge.make_line((30.347, 20.4123, 0), (30.5954, 20.9541, 0)),
    Edge.make_line((30.5954, 20.9541, 0), (30.8439, 21.4958, 0)),
    Edge.make_line((30.8439, 21.4958, 0), (31.0924, 22.0376, 0)),
    Edge.make_line((31.0924, 22.0376, 0), (30.8926, 22.5322, 0)),
    Edge.make_line((30.8926, 22.5322, 0), (30.5368, 23.0104, 0)),
    Edge.make_line((30.5368, 23.0104, 0), (30.1811, 23.4886, 0)),
    Edge.make_line((30.1811, 23.4886, 0), (29.8254, 23.9669, 0)),
    Edge.make_line((29.8254, 23.9669, 0), (29.4696, 24.4451, 0)),
    Edge.make_line((29.4696, 24.4451, 0), (29.1139, 24.9233, 0)),
    Edge.make_line((29.1139, 24.9233, 0), (28.8239, 24.769, 0)),
    Edge.make_line((28.8239, 24.769, 0), (28.574, 24.2279, 0)),
    Edge.make_line((28.574, 24.2279, 0), (28.3242, 23.6868, 0)),
    Edge.make_line((28.3242, 23.6868, 0), (28.0744, 23.1457, 0)),
    Edge.make_line((28.0744, 23.1457, 0), (27.8245, 22.6045, 0)),
    Edge.make_line((27.8245, 22.6045, 0), (27.5747, 22.0634, 0)),
    Edge.make_line((27.5747, 22.0634, 0), (27.3248, 21.5223, 0)),
    Edge.make_line((27.3248, 21.5223, 0), (27.075, 20.9811, 0)),
    Edge.make_line((27.075, 20.9811, 0), (26.8252, 20.44, 0)),
    Edge.make_line((26.8252, 20.44, 0), (26.5753, 19.8989, 0)),
    Edge.make_line((26.5753, 19.8989, 0), (26.3255, 19.3577, 0)),
    Edge.make_line((26.3255, 19.3577, 0), (26.0757, 18.8166, 0)),
    Edge.make_line((26.0757, 18.8166, 0), (25.8258, 18.2755, 0)),
    Edge.make_line((25.8258, 18.2755, 0), (25.576, 17.7344, 0)),
    Edge.make_line((25.576, 17.7344, 0), (25.3261, 17.1932, 0)),
    Edge.make_line((25.3261, 17.1932, 0), (25.0763, 16.6521, 0)),
]
sk_Sketch83_loft_49_wire = Wire(sk_Sketch83_loft_49_edges)
sk_Sketch83_loft_49_face = Face(sk_Sketch83_loft_49_wire)
sk_Sketch83_loft_49_face = Plane(origin=Vector(-84.5753, -21.191, -0.0), x_dir=Vector(-0.242313, 0.967094, 0.07755), z_dir=Vector(0.970015, 0.243045, 0.0)) * sk_Sketch83_loft_49_face
sk_Sketch82_loft_50_edges = [
    Edge.make_line((-35.4633, 1.9456, 0), (-35.5641, 1.2416, 0)),
    Edge.make_line((-35.5641, 1.2416, 0), (-35.6648, 0.5377, 0)),
    Edge.make_line((-35.6648, 0.5377, 0), (-35.7656, -0.1663, 0)),
    Edge.make_line((-35.7656, -0.1663, 0), (-35.828, -0.8744, 0)),
    Edge.make_line((-35.828, -0.8744, 0), (-35.8809, -1.5835, 0)),
    Edge.make_line((-35.8809, -1.5835, 0), (-35.9337, -2.2927, 0)),
    Edge.make_line((-35.9337, -2.2927, 0), (-35.9578, -3.003, 0)),
    Edge.make_line((-35.9578, -3.003, 0), (-35.9624, -3.7141, 0)),
    Edge.make_line((-35.9624, -3.7141, 0), (-35.967, -4.4252, 0)),
    Edge.make_line((-35.967, -4.4252, 0), (-35.9526, -5.1358, 0)),
    Edge.make_line((-35.9526, -5.1358, 0), (-35.9091, -5.8456, 0)),
    Edge.make_line((-35.9091, -5.8456, 0), (-35.8655, -6.5554, 0)),
    Edge.make_line((-35.8655, -6.5554, 0), (-35.8127, -7.2643, 0)),
    Edge.make_line((-35.8127, -7.2643, 0), (-35.7212, -7.9695, 0)),
    Edge.make_line((-35.7212, -7.9695, 0), (-35.6296, -8.6747, 0)),
    Edge.make_line((-35.6296, -8.6747, 0), (-35.5381, -9.38, 0)),
    Edge.make_line((-35.5381, -9.38, 0), (-35.3994, -10.0774, 0)),
    Edge.make_line((-35.3994, -10.0774, 0), (-35.2603, -10.7748, 0)),
    Edge.make_line((-35.2603, -10.7748, 0), (-35.1212, -11.4722, 0)),
    Edge.make_line((-35.1212, -11.4722, 0), (-34.6097, -11.906, 0)),
    Edge.make_line((-34.6097, -11.906, 0), (-33.9982, -12.269, 0)),
    Edge.make_line((-33.9982, -12.269, 0), (-33.3625, -12.5811, 0)),
    Edge.make_line((-33.3625, -12.5811, 0), (-32.6951, -12.8266, 0)),
    Edge.make_line((-32.6951, -12.8266, 0), (-32.025, -13.0621, 0)),
    Edge.make_line((-32.025, -13.0621, 0), (-31.3241, -13.1819, 0)),
    Edge.make_line((-31.3241, -13.1819, 0), (-30.6231, -13.3016, 0)),
    Edge.make_line((-30.6231, -13.3016, 0), (-29.9148, -13.3274, 0)),
    Edge.make_line((-29.9148, -13.3274, 0), (-29.2037, -13.3174, 0)),
    Edge.make_line((-29.2037, -13.3174, 0), (-28.4978, -13.2595, 0)),
    Edge.make_line((-28.4978, -13.2595, 0), (-27.8004, -13.1201, 0)),
    Edge.make_line((-27.8004, -13.1201, 0), (-27.1037, -12.9787, 0)),
    Edge.make_line((-27.1037, -12.9787, 0), (-26.4434, -12.7146, 0)),
    Edge.make_line((-26.4434, -12.7146, 0), (-25.7832, -12.4505, 0)),
    Edge.make_line((-25.7832, -12.4505, 0), (-25.1621, -12.1097, 0)),
    Edge.make_line((-25.1621, -12.1097, 0), (-24.561, -11.7297, 0)),
    Edge.make_line((-24.561, -11.7297, 0), (-24.2503, -11.2753, 0)),
    Edge.make_line((-24.2503, -11.2753, 0), (-24.5947, -10.6532, 0)),
    Edge.make_line((-24.5947, -10.6532, 0), (-24.9391, -10.031, 0)),
    Edge.make_line((-24.9391, -10.031, 0), (-25.2835, -9.4089, 0)),
    Edge.make_line((-25.2835, -9.4089, 0), (-25.6279, -8.7867, 0)),
    Edge.make_line((-25.6279, -8.7867, 0), (-25.9724, -8.1645, 0)),
    Edge.make_line((-25.9724, -8.1645, 0), (-26.3168, -7.5424, 0)),
    Edge.make_line((-26.3168, -7.5424, 0), (-26.6612, -6.9202, 0)),
    Edge.make_line((-26.6612, -6.9202, 0), (-27.0056, -6.2981, 0)),
    Edge.make_line((-27.0056, -6.2981, 0), (-27.35, -5.6759, 0)),
    Edge.make_line((-27.35, -5.6759, 0), (-27.6944, -5.0538, 0)),
    Edge.make_line((-27.6944, -5.0538, 0), (-28.0389, -4.4316, 0)),
    Edge.make_line((-28.0389, -4.4316, 0), (-28.3833, -3.8095, 0)),
    Edge.make_line((-28.3833, -3.8095, 0), (-28.7277, -3.1873, 0)),
    Edge.make_line((-28.7277, -3.1873, 0), (-29.0721, -2.5651, 0)),
    Edge.make_line((-29.0721, -2.5651, 0), (-29.4165, -1.943, 0)),
    Edge.make_line((-29.4165, -1.943, 0), (-29.7609, -1.3208, 0)),
    Edge.make_line((-29.7609, -1.3208, 0), (-30.1054, -0.6987, 0)),
    Edge.make_line((-30.1054, -0.6987, 0), (-30.4498, -0.0765, 0)),
    Edge.make_line((-30.4498, -0.0765, 0), (-30.7942, 0.5456, 0)),
    Edge.make_line((-30.7942, 0.5456, 0), (-31.1386, 1.1678, 0)),
    Edge.make_line((-31.1386, 1.1678, 0), (-31.483, 1.7899, 0)),
    Edge.make_line((-31.483, 1.7899, 0), (-31.8274, 2.4121, 0)),
    Edge.make_line((-31.8274, 2.4121, 0), (-32.1719, 3.0343, 0)),
    Edge.make_line((-32.1719, 3.0343, 0), (-32.7912, 2.9208, 0)),
    Edge.make_line((-32.7912, 2.9208, 0), (-33.4592, 2.677, 0)),
    Edge.make_line((-33.4592, 2.677, 0), (-34.1272, 2.4332, 0)),
    Edge.make_line((-34.1272, 2.4332, 0), (-34.7953, 2.1894, 0)),
    Edge.make_line((-34.7953, 2.1894, 0), (-35.4633, 1.9456, 0)),
]
sk_Sketch82_loft_50_wire = Wire(sk_Sketch82_loft_50_edges)
sk_Sketch82_loft_50_face = Face(sk_Sketch82_loft_50_wire)
sk_Sketch82_loft_50_face = Plane(origin=Vector(-67.2712, -21.7573, 21.3331), x_dir=Vector(0.309389, -0.950918, 0.005794), z_dir=Vector(-0.910911, -0.294613, 0.288869)) * sk_Sketch82_loft_50_face
with BuildLine() as _bl_Sweep3:
    ThreePointArc((-117.152, 16.5836, 0.0), (-118.279, 13.5593, 0.0), (-118.2526, 10.3321, 0.0))
path_Sweep3 = _bl_Sweep3.wires()[0]
_plane_Sweep3 = Plane(origin=Vector(-52.771, 64.1763, 0.0), x_dir=Vector(-0.41046115, -0.33751477, 0.84711595), z_dir=Vector(0.6351329, -0.77240288, -0.0))
with BuildSketch(_plane_Sweep3) as sk_Sketch93_50:
    with BuildLine():
        RadiusArc((58.9859, 59.119), (54.303, 66.0115), -29.6006)
        Line((54.303, 66.0115), (53.8304, 63.9746))
        RadiusArc((53.8304, 63.9746), (57.1813, 59.659), 30.5216)
        Line((57.1813, 59.659), (58.9859, 59.119))
    make_face()
_sketch_plane_52 = Plane(
    origin=Vector(-27.607, 0.0121, 47.7851),
    x_dir=Vector(-0.00044, -1.0, 0.0),
    z_dir=Vector(-0.500249, 0.00022, 0.865882),
)
with BuildSketch(_sketch_plane_52) as sk_Sketch96_52:
    with BuildLine():
        Line((-37.4251, -32.4024), (-37.4251, -32.5232))
        Line((-37.4251, -32.5232), (-38.6073, -29.9708))
        Line((-38.6073, -29.9708), (-38.8432, -27.8899))
        Line((-38.8432, -27.8899), (-38.2293, -27.2022))
        Line((-38.2293, -27.2022), (-37.4251, -32.4024))
    _edges_sk_Sketch96_52 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch96_52 = Wire.combine(_edges_sk_Sketch96_52)[0]
_w_sk_Sketch96_52 = _w_sk_Sketch96_52.moved(_sketch_plane_52.location)
_mf_sk_Sketch96_52 = BRepBuilderAPI_MakeFace(_sketch_plane_52.wrapped, _w_sk_Sketch96_52.wrapped, True)
_face_sk_Sketch96_52 = Face(_mf_sk_Sketch96_52.Face())
_sketch_plane_53 = Plane(
    origin=Vector(-24.7304, -20.3355, 51.0393),
    x_dir=Vector(0.635133, -0.772403, -0.0),
    z_dir=Vector(-0.410461, -0.337515, 0.847116),
)
with BuildSketch(_sketch_plane_53) as sk_Sketch95_53:
    with BuildLine():
        Line((-81.1292, -61.6022), (-86.6853, -61.6022))
        Line((-86.6853, -61.6022), (-86.6853, -56.3349))
        Line((-86.6853, -56.3349), (-81.1292, -56.3349))
        Line((-81.1292, -56.3349), (-81.1292, -61.6022))
    _edges_sk_Sketch95_53 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch95_53 = Wire.combine(_edges_sk_Sketch95_53)[0]
_w_sk_Sketch95_53 = _w_sk_Sketch95_53.moved(_sketch_plane_53.location)
_mf_sk_Sketch95_53 = BRepBuilderAPI_MakeFace(_sketch_plane_53.wrapped, _w_sk_Sketch95_53.wrapped, True)
_face_sk_Sketch95_53 = Face(_mf_sk_Sketch95_53.Face())
_sketch_plane_54 = Plane(
    origin=Vector(0.0, 0.0, 10.9074),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_sketch_plane_54) as sk_Sketch97_54:
    with BuildLine():
        RadiusArc((75.7266, -12.2378), (79.3775, -5.6878), -63.0607)
        Line((79.3775, -5.6878), (73.0, -5.0))
        RadiusArc((73.0, -5.0), (70.7323, -9.8818), 22.963)
        Line((70.7323, -9.8818), (75.7266, -12.2378))
    _edges_sk_Sketch97_54 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch97_54 = Wire.combine(_edges_sk_Sketch97_54)[0]
_w_sk_Sketch97_54 = _w_sk_Sketch97_54.moved(_sketch_plane_54.location)
_mf_sk_Sketch97_54 = BRepBuilderAPI_MakeFace(_sketch_plane_54.wrapped, _w_sk_Sketch97_54.wrapped, True)
_face_sk_Sketch97_54 = Face(_mf_sk_Sketch97_54.Face())
_sketch_plane_55 = Plane(
    origin=Vector(0.0, 0.0, 10.9074),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_sketch_plane_55) as sk_Sketch102_55:
    with BuildLine():
        RadiusArc((80.3, -8.9), (80.3257, -8.2627), -0.5398)
        RadiusArc((80.3257, -8.2627), (79.0, -8.0), -1.0642)
        RadiusArc((79.0, -8.0), (77.0619, -7.4053), -6.147)
        RadiusArc((77.0619, -7.4053), (74.3676, -9.9088), -20.826)
        Line((74.3676, -9.9088), (79.3097, -8.8736))
        RadiusArc((79.3097, -8.8736), (80.3, -8.9), -0.6407)
    _edges_sk_Sketch102_55 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch102_55 = Wire.combine(_edges_sk_Sketch102_55)[0]
_w_sk_Sketch102_55 = _w_sk_Sketch102_55.moved(_sketch_plane_55.location)
_mf_sk_Sketch102_55 = BRepBuilderAPI_MakeFace(_sketch_plane_55.wrapped, _w_sk_Sketch102_55.wrapped, True)
_face_sk_Sketch102_55 = Face(_mf_sk_Sketch102_55.Face())
_sketch_plane_56 = Plane(
    origin=Vector(4.1039, 34.3254, 2.3623),
    x_dir=Vector(0.992928, -0.118715, 0.0),
    z_dir=Vector(-0.118438, -0.990618, -0.068174),
)
with BuildSketch(_sketch_plane_56) as sk_Sketch103_56:
    with BuildLine():
        RadiusArc((-54.9014, 32.1637), (-55.399, 29.6872), -18.8677)
        RadiusArc((-55.399, 29.6872), (-48.3505, 34.8163), 39.174)
        Line((-48.3505, 34.8163), (-47.9625, 34.8155))
        Line((-47.9625, 34.8155), (-47.5877, 35.1184))
        Line((-47.5877, 35.1184), (-46.9552, 35.3898))
        Line((-46.9552, 35.3898), (-46.6865, 35.8996))
        Line((-46.6865, 35.8996), (-45.9191, 36.4751))
        Line((-45.9191, 36.4751), (-46.0117, 37.3888))
        RadiusArc((-46.0117, 37.3888), (-54.9014, 32.1637), 770.1299)
    _edges_sk_Sketch103_56 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch103_56 = Wire.combine(_edges_sk_Sketch103_56)[0]
_w_sk_Sketch103_56 = _w_sk_Sketch103_56.moved(_sketch_plane_56.location)
_mf_sk_Sketch103_56 = BRepBuilderAPI_MakeFace(_sketch_plane_56.wrapped, _w_sk_Sketch103_56.wrapped, True)
_face_sk_Sketch103_56 = Face(_mf_sk_Sketch103_56.Face())
_sketch_plane_57 = Plane(
    origin=Vector(-3.8609, 41.0645, -2.241),
    x_dir=Vector(0.995609, 0.093609, 0.0),
    z_dir=Vector(0.093471, -0.994143, 0.054254),
)
with BuildSketch(_sketch_plane_57) as sk_Sketch104_57:
    with BuildLine():
        Line((-47.2783, 34.2939), (-47.7516, 36.2157))
        Line((-47.7516, 36.2157), (-46.8276, 36.7464))
        Line((-46.8276, 36.7464), (-47.2783, 34.2939))
    _edges_sk_Sketch104_57 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch104_57 = Wire.combine(_edges_sk_Sketch104_57)[0]
_w_sk_Sketch104_57 = _w_sk_Sketch104_57.moved(_sketch_plane_57.location)
_mf_sk_Sketch104_57 = BRepBuilderAPI_MakeFace(_sketch_plane_57.wrapped, _w_sk_Sketch104_57.wrapped, True)
_face_sk_Sketch104_57 = Face(_mf_sk_Sketch104_57.Face())
_sketch_plane_58 = Plane(
    origin=Vector(-7.5419, 42.4373, -4.368),
    x_dir=Vector(0.501175, 0.0, -0.865346),
    z_dir=Vector(0.174086, -0.979555, 0.100824),
)
with BuildSketch(_sketch_plane_58) as sk_Sketch105_58:
    with BuildLine():
        RadiusArc((-55.2174, -25.8407), (-53.306, -19.8277), -50.0675)
        Line((-53.306, -19.8277), (-54.9575, -19.1619))
        Line((-54.9575, -19.1619), (-55.2174, -25.8407))
    _edges_sk_Sketch105_58 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch105_58 = Wire.combine(_edges_sk_Sketch105_58)[0]
_w_sk_Sketch105_58 = _w_sk_Sketch105_58.moved(_sketch_plane_58.location)
_mf_sk_Sketch105_58 = BRepBuilderAPI_MakeFace(_sketch_plane_58.wrapped, _w_sk_Sketch105_58.wrapped, True)
_face_sk_Sketch105_58 = Face(_mf_sk_Sketch105_58.Face())
_sketch_plane_59 = Plane(
    origin=Vector(0.0, 0.0, 12.9074),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_sketch_plane_59) as sk_Sketch106_59:
    with BuildLine():
        RadiusArc((112.1327, -21.3512), (115.9745, -16.5345), -90.7442)
        RadiusArc((115.9745, -16.5345), (116.5215, -16.5573), -1.4023)
        RadiusArc((116.5215, -16.5573), (117.563, -14.2406), -14.4808)
        RadiusArc((117.563, -14.2406), (118.1412, -14.3227), 1.52)
        RadiusArc((118.1412, -14.3227), (112.5029, -21.8277), 14.7749)
        Line((112.5029, -21.8277), (112.1327, -21.3512))
    _edges_sk_Sketch106_59 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch106_59 = Wire.combine(_edges_sk_Sketch106_59)[0]
_w_sk_Sketch106_59 = _w_sk_Sketch106_59.moved(_sketch_plane_59.location)
_mf_sk_Sketch106_59 = BRepBuilderAPI_MakeFace(_sketch_plane_59.wrapped, _w_sk_Sketch106_59.wrapped, True)
_face_sk_Sketch106_59 = Face(_mf_sk_Sketch106_59.Face())
_wp_60 = Plane(
    origin=Vector(-51.6877, 66.5377, 0.0),
    x_dir=Vector(0.789719, 0.613469, 0.0),
    z_dir=Vector(0.613469, -0.789719, 0.0),
)
with BuildSketch(_wp_60) as sk_Sketch107_60:
    with BuildLine():
        Line((-75.455, 10.9074), (-69.3133, 16.1896))
        RadiusArc((-69.3133, 16.1896), (-63.9378, 20.5204), -31.2417)
        Line((-63.9378, 20.5204), (-64.2755, 11.0))
        Line((-64.2755, 11.0), (-75.455, 10.9074))
    _edges_sk_Sketch107_60 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch107_60 = Wire.combine(_edges_sk_Sketch107_60)[0]
_w_sk_Sketch107_60 = _w_sk_Sketch107_60.moved(_wp_60.location)
_mf_sk_Sketch107_60 = BRepBuilderAPI_MakeFace(_wp_60.wrapped, _w_sk_Sketch107_60.wrapped, True)
_face_sk_Sketch107_60 = Face(_mf_sk_Sketch107_60.Face())
_wp_61 = Plane(
    origin=Vector(0.0, 0.0, 6.9074),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_wp_61) as sk_Sketch108_61:
    with BuildLine():
        Line((-79.5495, -6.4094), (-77.4309, -6.4094))
        Line((-77.4309, -6.4094), (-77.4309, -7.8477))
        Line((-77.4309, -7.8477), (-79.5495, -7.8477))
        Line((-79.5495, -7.8477), (-79.5495, -6.4094))
    _edges_sk_Sketch108_61 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch108_61 = Wire.combine(_edges_sk_Sketch108_61)[0]
_w_sk_Sketch108_61 = _w_sk_Sketch108_61.moved(_wp_61.location)
_mf_sk_Sketch108_61 = BRepBuilderAPI_MakeFace(_wp_61.wrapped, _w_sk_Sketch108_61.wrapped, True)
_face_sk_Sketch108_61 = Face(_mf_sk_Sketch108_61.Face())
_wp_62 = Plane(
    origin=Vector(0.0, 0.0, 10.9074),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_wp_62) as sk_Sketch109_62:
    with BuildLine():
        Line((96.6153, -40.6301), (97.79, -36.6135))
        Line((97.79, -36.6135), (97.4093, -40.5637))
        Line((97.4093, -40.5637), (97.2889, -40.5151))
        Line((97.2889, -40.5151), (97.3, -40.4))
        Line((97.3, -40.4), (96.6153, -40.6301))
    _edges_sk_Sketch109_62 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch109_62 = Wire.combine(_edges_sk_Sketch109_62)[0]
_w_sk_Sketch109_62 = _w_sk_Sketch109_62.moved(_wp_62.location)
_mf_sk_Sketch109_62 = BRepBuilderAPI_MakeFace(_wp_62.wrapped, _w_sk_Sketch109_62.wrapped, True)
_face_sk_Sketch109_62 = Face(_mf_sk_Sketch109_62.Face())
_wp_63 = Plane(
    origin=Vector(0.0, 0.0, 10.9074),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_wp_63) as sk_Sketch109_63:
    with BuildLine():
        Line((98.8029, -35.6976), (98.7156, -34.4797))
        Line((98.7156, -34.4797), (98.8126, -33.5499))
        Line((98.8126, -33.5499), (98.1211, -33.4831))
        Line((98.1211, -33.4831), (97.79, -36.6135))
        Line((97.79, -36.6135), (98.8029, -35.6976))
    _edges_sk_Sketch109_63 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch109_63 = Wire.combine(_edges_sk_Sketch109_63)[0]
_w_sk_Sketch109_63 = _w_sk_Sketch109_63.moved(_wp_63.location)
_mf_sk_Sketch109_63 = BRepBuilderAPI_MakeFace(_wp_63.wrapped, _w_sk_Sketch109_63.wrapped, True)
_face_sk_Sketch109_63 = Face(_mf_sk_Sketch109_63.Face())
_wp_64 = Plane(
    origin=Vector(-24.7305, -20.3355, 51.0393),
    x_dir=Vector(0.635133, -0.772403, -0.0),
    z_dir=Vector(-0.410461, -0.337515, 0.847116),
)
with BuildSketch(_wp_64) as sk_Sketch110_64:
    with BuildLine():
        Line((-80.7059, -55.3713), (-80.885, -57.5562))
        Line((-80.885, -57.5562), (-66.9436, -58.2963))
        Line((-66.9436, -58.2963), (-66.8278, -56.1367))
        Line((-66.8278, -56.1367), (-80.7059, -55.3713))
    _edges_sk_Sketch110_64 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_w_sk_Sketch110_64 = Wire.combine(_edges_sk_Sketch110_64)[0]
_w_sk_Sketch110_64 = _w_sk_Sketch110_64.moved(_wp_64.location)
_mf_sk_Sketch110_64 = BRepBuilderAPI_MakeFace(_wp_64.wrapped, _w_sk_Sketch110_64.wrapped, True)
_face_sk_Sketch110_64 = Face(_mf_sk_Sketch110_64.Face())
sk_loft1_loft_65_edges = [
    Edge.make_line((77.0602, 3.5786, 0), (77.3392, 3.1289, 0)),
    Edge.make_line((77.3392, 3.1289, 0), (77.6228, 2.6828, 0)),
    Edge.make_line((77.6228, 2.6828, 0), (78.0058, 2.3174, 0)),
    Edge.make_line((78.0058, 2.3174, 0), (78.3887, 1.9521, 0)),
    Edge.make_line((78.3887, 1.9521, 0), (78.7717, 1.5868, 0)),
    Edge.make_line((78.7717, 1.5868, 0), (79.1547, 1.2214, 0)),
    Edge.make_line((79.1547, 1.2214, 0), (79.5377, 0.8561, 0)),
    Edge.make_line((79.5377, 0.8561, 0), (79.9206, 0.4908, 0)),
    Edge.make_line((79.9206, 0.4908, 0), (80.3036, 0.1254, 0)),
    Edge.make_line((80.3036, 0.1254, 0), (80.6866, -0.2399, 0)),
    Edge.make_line((80.6866, -0.2399, 0), (81.0695, -0.6052, 0)),
    Edge.make_line((81.0695, -0.6052, 0), (81.4525, -0.9706, 0)),
    Edge.make_line((81.4525, -0.9706, 0), (81.8355, -1.3359, 0)),
    Edge.make_line((81.8355, -1.3359, 0), (82.2184, -1.7013, 0)),
    Edge.make_line((82.2184, -1.7013, 0), (82.6014, -2.0666, 0)),
    Edge.make_line((82.6014, -2.0666, 0), (82.9844, -2.4319, 0)),
    Edge.make_line((82.9844, -2.4319, 0), (83.3674, -2.7973, 0)),
    Edge.make_line((83.3674, -2.7973, 0), (83.7503, -3.1626, 0)),
    Edge.make_line((83.7503, -3.1626, 0), (84.1333, -3.5279, 0)),
    Edge.make_line((84.1333, -3.5279, 0), (84.5163, -3.8933, 0)),
    Edge.make_line((84.5163, -3.8933, 0), (84.947, -4.1539, 0)),
    Edge.make_line((84.947, -4.1539, 0), (85.474, -4.2035, 0)),
    Edge.make_line((85.474, -4.2035, 0), (86.0009, -4.2531, 0)),
    Edge.make_line((86.0009, -4.2531, 0), (86.5279, -4.3027, 0)),
    Edge.make_line((86.5279, -4.3027, 0), (87.0563, -4.3207, 0)),
    Edge.make_line((87.0563, -4.3207, 0), (87.5855, -4.3195, 0)),
    Edge.make_line((87.5855, -4.3195, 0), (88.1148, -4.3183, 0)),
    Edge.make_line((88.1148, -4.3183, 0), (88.6441, -4.3172, 0)),
    Edge.make_line((88.6441, -4.3172, 0), (89.1734, -4.316, 0)),
    Edge.make_line((89.1734, -4.316, 0), (89.453, -4.0655, 0)),
    Edge.make_line((89.453, -4.0655, 0), (89.4537, -3.5362, 0)),
    Edge.make_line((89.4537, -3.5362, 0), (89.4543, -3.0069, 0)),
    Edge.make_line((89.4543, -3.0069, 0), (89.455, -2.4776, 0)),
    Edge.make_line((89.455, -2.4776, 0), (89.4557, -1.9483, 0)),
    Edge.make_line((89.4557, -1.9483, 0), (89.4563, -1.4191, 0)),
    Edge.make_line((89.4563, -1.4191, 0), (89.3986, -0.9243, 0)),
    Edge.make_line((89.3986, -0.9243, 0), (88.9344, -0.6701, 0)),
    Edge.make_line((88.9344, -0.6701, 0), (88.4702, -0.4158, 0)),
    Edge.make_line((88.4702, -0.4158, 0), (88.0059, -0.1615, 0)),
    Edge.make_line((88.0059, -0.1615, 0), (87.5417, 0.0927, 0)),
    Edge.make_line((87.5417, 0.0927, 0), (87.0775, 0.347, 0)),
    Edge.make_line((87.0775, 0.347, 0), (86.6133, 0.6013, 0)),
    Edge.make_line((86.6133, 0.6013, 0), (86.1491, 0.8555, 0)),
    Edge.make_line((86.1491, 0.8555, 0), (85.6849, 1.1098, 0)),
    Edge.make_line((85.6849, 1.1098, 0), (85.2207, 1.3641, 0)),
    Edge.make_line((85.2207, 1.3641, 0), (84.7565, 1.6183, 0)),
    Edge.make_line((84.7565, 1.6183, 0), (84.2923, 1.8726, 0)),
    Edge.make_line((84.2923, 1.8726, 0), (83.8281, 2.1268, 0)),
    Edge.make_line((83.8281, 2.1268, 0), (83.3639, 2.3811, 0)),
    Edge.make_line((83.3639, 2.3811, 0), (82.8997, 2.6354, 0)),
    Edge.make_line((82.8997, 2.6354, 0), (82.4355, 2.8896, 0)),
    Edge.make_line((82.4355, 2.8896, 0), (81.9713, 3.1439, 0)),
    Edge.make_line((81.9713, 3.1439, 0), (81.5071, 3.3982, 0)),
    Edge.make_line((81.5071, 3.3982, 0), (81.0429, 3.6524, 0)),
    Edge.make_line((81.0429, 3.6524, 0), (80.5787, 3.9067, 0)),
    Edge.make_line((80.5787, 3.9067, 0), (80.2899, 4.3429, 0)),
    Edge.make_line((80.2899, 4.3429, 0), (80.019, 4.7976, 0)),
    Edge.make_line((80.019, 4.7976, 0), (79.7481, 5.2523, 0)),
    Edge.make_line((79.7481, 5.2523, 0), (79.3026, 4.9839, 0)),
    Edge.make_line((79.3026, 4.9839, 0), (78.8542, 4.7028, 0)),
    Edge.make_line((78.8542, 4.7028, 0), (78.4057, 4.4218, 0)),
    Edge.make_line((78.4057, 4.4218, 0), (77.9572, 4.1407, 0)),
    Edge.make_line((77.9572, 4.1407, 0), (77.5087, 3.8597, 0)),
    Edge.make_line((77.5087, 3.8597, 0), (77.0602, 3.5786, 0)),
]
sk_loft1_loft_65_wire = Wire(sk_loft1_loft_65_edges)
sk_loft1_loft_65_face = Face(sk_loft1_loft_65_wire)
sk_loft1_loft_65_face = Plane(origin=Vector(0.0, 0.0, 10.9074), x_dir=Vector(-1.0, 0.0, 0.0), z_dir=Vector(0.0, 0.0, 1.0)) * sk_loft1_loft_65_face
with BuildSketch(Plane.XY) as sk_loft2_loft_66:
    pass
with BuildSketch(Plane.XY) as sk_loft3_loft_67:
    pass
with BuildPart() as part:
    _face = _face_sk_Sketch1
    _vec = Vector(-0.342038, -0.340204, 0.87594) * -4.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    _face = _face_sk_Sketch2_2
    _vec = Vector(-0.342038, -0.340204, 0.87594) * -5.0
    _solid = Solid.extrude(_face, _vec)
    _result_Extrude2 = cut_solids(part.part, _solid)
    if _result_Extrude2 is not None: part.part = _result_Extrude2
    _face = _face_sk_Sketch4_3
    _vec = Vector(-0.410461, -0.337515, 0.847116) * -4.0142
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch7_4
    _vec = Vector(-0.409915, -0.339111, 0.846743) * -11.0
    _solid = Solid.extrude(_face, _vec)
    _result_Extrude4 = cut_solids(part.part, _solid)
    if _result_Extrude4 is not None: part.part = _result_Extrude4
    _vec_sym = Vector(-0.409915, -0.339111, 0.846743) * 11.0
    _solid_sym = Solid.extrude(_face, _vec_sym)
    add(_solid_sym, mode=Mode.SUBTRACT)
    _face = _face_sk_Sketch9_5
    _vec = Vector(0.510815, -0.849755, -0.13032) * -0.501
    _solid = Solid.extrude(_face, _vec)
    _result_Extrude6 = cut_solids(part.part, _solid)
    if _result_Extrude6 is not None: part.part = _result_Extrude6
    _face = _face_sk_Sketch3_6
    _vec = Vector(0.0, 0.0, -1.0) * -4.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    extrude(sk_Sketch10_7.sketch, amount=-4.0, mode=Mode.SUBTRACT)
    extrude(sk_Sketch11_8.sketch, amount=-4.0, mode=Mode.SUBTRACT)
    _face = _face_sk_Sketch13_9
    _vec = Vector(0.0, 0.0, 1.0) * 4.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _vec2 = Vector(0.0, 0.0, 1.0) * -0.1
    _solid2 = Solid.extrude(_face, _vec2)
    add(_solid2, mode=Mode.ADD)
    _face = _face_sk_Sketch12_10
    _vec = Vector(0.499995, 0.000273, -0.866028) * -5.996
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch17_11
    _vec = Vector(0.44, -0.08, -0.894427) * 1.0
    _solid = Solid.extrude(_face, _vec)
    _result_Extrude13 = cut_solids(part.part, _solid)
    if _result_Extrude13 is not None: part.part = _result_Extrude13
    _loft_solid = Solid.make_loft([sk_Sketch16_loft_12_face.outer_wire(), sk_Sketch18_loft_13_face.outer_wire()])
    add(_loft_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch19_14
    _vec = Vector(0.230537, 0.208442, 0.950476) * 2.0
    _solid = Solid.extrude(_face, _vec)
    _result_Extrude14 = cut_solids(part.part, _solid)
    if _result_Extrude14 is not None: part.part = _result_Extrude14
    extrude(sk_Sketch20_15.sketch, amount=-10.0, mode=Mode.SUBTRACT)
    add(_ext_sk_Sketch20_15, mode=Mode.SUBTRACT)
    _custom_axis = Axis(
        Vector(-53.7889, 13.0994, 39.5671),
        Vector(-0.500044, 0.000127, 0.866),
    )
    revolve(sk_Sketch22_15.sketch.faces(), axis=_custom_axis, mode=Mode.SUBTRACT)
    _face = _face_sk_Sketch23_17
    _vec = Vector(-0.499995, -0.000273, 0.866028) * -5.996
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch24_18
    _vec = Vector(-0.500249, 0.00022, 0.865882) * -5.988
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch31_19
    _vec = Vector(-0.788736, -0.402759, -0.464413) * 3.2
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch32_20
    _vec = Vector(0.688131, 0.494902, 0.53061) * 1.435
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch33_21
    _vec = Vector(0.410461, 0.337515, -0.847116) * 1.0
    _solid = Solid.extrude(_face, _vec)
    _result_Extrude20 = cut_solids(part.part, _solid)
    if _result_Extrude20 is not None: part.part = _result_Extrude20
    _face = _face_sk_Sketch34_22
    _vec = Vector(-0.342038, -0.340204, 0.87594) * 0.6
    _solid = Solid.extrude(_face, _vec)
    _result_Extrude21 = cut_solids(part.part, _solid)
    if _result_Extrude21 is not None: part.part = _result_Extrude21
    _face = _face_sk_Sketch35_23
    _vec = Vector(-0.410471, -0.349747, 0.842135) * 2.0
    _solid = Solid.extrude(_face, _vec)
    _result_Extrude22 = cut_solids(part.part, _solid)
    if _result_Extrude22 is not None: part.part = _result_Extrude22
    _face = _face_sk_Sketch37_24
    _vec = Vector(-0.196733, -0.237016, 0.951378) * -1.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch38_25
    _vec = Vector(-0.410471, -0.349747, 0.842135) * 3.0
    _solid = Solid.extrude(_face, _vec)
    _result_Extrude25 = cut_solids(part.part, _solid)
    if _result_Extrude25 is not None: part.part = _result_Extrude25
    try:
        from OCP.BRepOffsetAPI import BRepOffsetAPI_MakePipeShell
        from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid, BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeFace
        from OCP.ShapeFix import ShapeFix_Solid
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopAbs import TopAbs_SHELL, TopAbs_WIRE, TopAbs_EDGE
        from OCP.TopoDS import TopoDS
        from OCP.ShapeAnalysis import ShapeAnalysis_FreeBounds
        from OCP.BRepAdaptor import BRepAdaptor_Curve
        from OCP.gp import gp_Pln, gp_Ax3, gp_Dir, gp_Pnt
        import numpy as _np
        _profile_face = sk_Sketch41_25.sketch.faces()[0]
        _occ_wire = None
        _wire_exp = TopExp_Explorer(_profile_face.wrapped, TopAbs_WIRE)
        if _wire_exp.More():
            _occ_wire = TopoDS.Wire_s(_wire_exp.Current())
        _path_wire = path_Sweep2
        def _make_pipe_solid(_wire, reverse=False):
            _w = _wire.Reversed() if reverse else _wire
            _pipe = BRepOffsetAPI_MakePipeShell(_path_wire.wrapped)
            _pipe.Add(_w)
            _pipe.Build()
            if not _pipe.IsDone(): return None
            if _pipe.MakeSolid(): return Solid(_pipe.Shape())
            return None
        def _fit_plane_cap(wire):
            _pts = []
            _ee = TopExp_Explorer(wire, TopAbs_EDGE)
            while _ee.More():
                _c = BRepAdaptor_Curve(TopoDS.Edge_s(_ee.Current()))
                _t = (_c.FirstParameter() + _c.LastParameter()) / 2.0
                _p = _c.Value(_t)
                _pts.append([_p.X(), _p.Y(), _p.Z()])
                _ee.Next()
            if len(_pts) < 3: return None
            _pts = _np.array(_pts)
            _cen = _pts.mean(axis=0)
            _, _, _vh = _np.linalg.svd(_pts - _cen)
            _n = _vh[-1]; _n /= _np.linalg.norm(_n)
            _x = _pts[0] - _cen; _x -= _np.dot(_x, _n) * _n
            if _np.linalg.norm(_x) < 1e-6: _x = _pts[1] - _cen; _x -= _np.dot(_x, _n) * _n
            _x /= _np.linalg.norm(_x)
            _ax = gp_Ax3(gp_Pnt(*_cen.tolist()), gp_Dir(*_n.tolist()), gp_Dir(*_x.tolist()))
            _mf = BRepBuilderAPI_MakeFace(gp_Pln(_ax), wire)
            return _mf.Face() if _mf.IsDone() else None
        _solid = _make_pipe_solid(_occ_wire) if _occ_wire else None
        if _solid is None and _occ_wire:
            _solid = _make_pipe_solid(_occ_wire, reverse=True)
        if _solid is None:
            _sweep_shell = Solid.sweep(sk_Sketch41_25.sketch.faces()[0], path_Sweep2)
            _sa = ShapeAnalysis_FreeBounds(_sweep_shell.wrapped)
            _cw_exp = TopExp_Explorer(_sa.GetClosedWires(), TopAbs_WIRE)
            _caps = []
            while _cw_exp.More():
                _w = TopoDS.Wire_s(_cw_exp.Current())
                _mf = BRepBuilderAPI_MakeFace(_w, True)
                if _mf.IsDone(): _caps.append(_mf.Face())
                else:
                    _fc = _fit_plane_cap(_w)
                    if _fc is not None: _caps.append(_fc)
                _cw_exp.Next()
            _sew = BRepBuilderAPI_Sewing(0.1)
            _sew.Add(_sweep_shell.wrapped)
            for _fc in _caps: _sew.Add(_fc)
            _sew.Perform()
            _mk = BRepBuilderAPI_MakeSolid()
            _exp = TopExp_Explorer(_sew.SewedShape(), TopAbs_SHELL)
            while _exp.More(): _mk.Add(TopoDS.Shell_s(_exp.Current())); _exp.Next()
            _mk.Build()
            if _mk.IsDone():
                _fix = ShapeFix_Solid(_mk.Solid())
                _fix.Perform()
                _solid = Solid(_fix.Shape())
            else:
                _solid = _sweep_shell
                print('WARNING: Sweep2 sweep — all solid attempts failed, result is Shell')
        from OCP.TopAbs import TopAbs_SHELL as _TS_SHELL, TopAbs_SOLID as _TS_SOLID
        if hasattr(_solid, 'wrapped') and _solid.wrapped.ShapeType() != _TS_SOLID:
            try:
                from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid as _MkSol2
                _mk2 = _MkSol2()
                _exp2 = TopExp_Explorer(_solid.wrapped, _TS_SHELL)
                while _exp2.More(): _mk2.Add(TopoDS.Shell_s(_exp2.Current())); _exp2.Next()
                _mk2.Build()
                if _mk2.IsDone(): _solid = Solid(_mk2.Shape())
            except Exception as _coerce_err:
                print('WARNING: Sweep2 Shell→Solid coercion failed:', _coerce_err)
        add(_solid, mode=Mode.ADD)
    except Exception as _sweep_err:
        print('WARNING: Sweep2 sweep failed:', _sweep_err)
    _loft_solid = Solid.make_loft([sk_Sketch53_loft_27_face.outer_wire(), sk_Sketch52_loft_28_face.outer_wire()])
    add(_loft_solid, mode=Mode.ADD)
    pass
    _face = _face_sk_Sketch58_29
    _vec = Vector(0.613469, -0.789719, -1e-06) * -2.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch59_30
    _vec = Vector(-0.997438, -0.071538, 0.0) * -2.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch60_31
    _vec = Vector(-0.983378, 0.18157, 0.0) * -2.114
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch61_32
    _vec = Vector(-0.997897, 0.064814, 0.0) * -2.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch62_33
    _vec = Vector(-0.997897, 0.064814, 0.0) * -2.197
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch63_34
    _vec = Vector(0.878205, -0.478284, 0.0) * -1.976
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch66_35
    _vec = Vector(0.945848, -0.32461, 0.0) * -1.903
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch67_36
    _vec = Vector(0.0, 0.0, -1.0) * 20.0
    _solid = Solid.extrude(_face, _vec)
    _result_Extrude46 = cut_solids(part.part, _solid)
    if _result_Extrude46 is not None: part.part = _result_Extrude46
    _face = _face_sk_Sketch69_37
    _vec = Vector(0.89557, 0.439601, 0.068587) * -21.0
    _solid = Solid.extrude(_face, _vec)
    _result_Extrude48 = cut_solids(part.part, _solid)
    if _result_Extrude48 is not None: part.part = _result_Extrude48
    _face = _face_sk_Sketch70_38
    _vec = Vector(0.669115, -0.743159, 0.0) * -1.912
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch71_39
    _vec = Vector(0.990074, -0.140548, -0.0) * -1.942
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch72_40
    _vec = Vector(0.996713, 0.081019, 0.0) * -2.029
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch73_41
    _vec = Vector(0.0, 0.0, 1.0) * 2.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _loft_solid = Solid.make_loft([sk_Sketch77_loft_42_face.outer_wire(), sk_Sketch78_loft_43_face.outer_wire()])
    add(_loft_solid, mode=Mode.ADD)
    _loft_solid = Solid.make_loft([sk_Sketch81_loft_44_face.outer_wire(), sk_Sketch79_loft_45_face.outer_wire()])
    add(_loft_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch84_46
    _vec = Vector(0.878205, -0.478284, -0.0) * 1.6
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    pass
    pass
    pass
    _loft_solid = Solid.make_loft([sk_Sketch88_loft_47_face.outer_wire(), sk_Sketch87_loft_48_face.outer_wire()])
    add(_loft_solid, mode=Mode.ADD)
    pass
    pass
    _loft_solid = Solid.make_loft([sk_Sketch83_loft_49_face.outer_wire(), sk_Sketch82_loft_50_face.outer_wire()])
    add(_loft_solid, mode=Mode.ADD)
    try:
        from OCP.BRepOffsetAPI import BRepOffsetAPI_MakePipeShell
        from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid, BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeFace
        from OCP.ShapeFix import ShapeFix_Solid
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopAbs import TopAbs_SHELL, TopAbs_WIRE, TopAbs_EDGE
        from OCP.TopoDS import TopoDS
        from OCP.ShapeAnalysis import ShapeAnalysis_FreeBounds
        from OCP.BRepAdaptor import BRepAdaptor_Curve
        from OCP.gp import gp_Pln, gp_Ax3, gp_Dir, gp_Pnt
        import numpy as _np
        _profile_face = sk_Sketch93_50.sketch.faces()[0]
        _occ_wire = None
        _wire_exp = TopExp_Explorer(_profile_face.wrapped, TopAbs_WIRE)
        if _wire_exp.More():
            _occ_wire = TopoDS.Wire_s(_wire_exp.Current())
        _path_wire = path_Sweep3
        def _make_pipe_solid(_wire, reverse=False):
            _w = _wire.Reversed() if reverse else _wire
            _pipe = BRepOffsetAPI_MakePipeShell(_path_wire.wrapped)
            _pipe.Add(_w)
            _pipe.Build()
            if not _pipe.IsDone(): return None
            if _pipe.MakeSolid(): return Solid(_pipe.Shape())
            return None
        def _fit_plane_cap(wire):
            _pts = []
            _ee = TopExp_Explorer(wire, TopAbs_EDGE)
            while _ee.More():
                _c = BRepAdaptor_Curve(TopoDS.Edge_s(_ee.Current()))
                _t = (_c.FirstParameter() + _c.LastParameter()) / 2.0
                _p = _c.Value(_t)
                _pts.append([_p.X(), _p.Y(), _p.Z()])
                _ee.Next()
            if len(_pts) < 3: return None
            _pts = _np.array(_pts)
            _cen = _pts.mean(axis=0)
            _, _, _vh = _np.linalg.svd(_pts - _cen)
            _n = _vh[-1]; _n /= _np.linalg.norm(_n)
            _x = _pts[0] - _cen; _x -= _np.dot(_x, _n) * _n
            if _np.linalg.norm(_x) < 1e-6: _x = _pts[1] - _cen; _x -= _np.dot(_x, _n) * _n
            _x /= _np.linalg.norm(_x)
            _ax = gp_Ax3(gp_Pnt(*_cen.tolist()), gp_Dir(*_n.tolist()), gp_Dir(*_x.tolist()))
            _mf = BRepBuilderAPI_MakeFace(gp_Pln(_ax), wire)
            return _mf.Face() if _mf.IsDone() else None
        _solid = _make_pipe_solid(_occ_wire) if _occ_wire else None
        if _solid is None and _occ_wire:
            _solid = _make_pipe_solid(_occ_wire, reverse=True)
        if _solid is None:
            _sweep_shell = Solid.sweep(sk_Sketch93_50.sketch.faces()[0], path_Sweep3)
            _sa = ShapeAnalysis_FreeBounds(_sweep_shell.wrapped)
            _cw_exp = TopExp_Explorer(_sa.GetClosedWires(), TopAbs_WIRE)
            _caps = []
            while _cw_exp.More():
                _w = TopoDS.Wire_s(_cw_exp.Current())
                _mf = BRepBuilderAPI_MakeFace(_w, True)
                if _mf.IsDone(): _caps.append(_mf.Face())
                else:
                    _fc = _fit_plane_cap(_w)
                    if _fc is not None: _caps.append(_fc)
                _cw_exp.Next()
            _sew = BRepBuilderAPI_Sewing(0.1)
            _sew.Add(_sweep_shell.wrapped)
            for _fc in _caps: _sew.Add(_fc)
            _sew.Perform()
            _mk = BRepBuilderAPI_MakeSolid()
            _exp = TopExp_Explorer(_sew.SewedShape(), TopAbs_SHELL)
            while _exp.More(): _mk.Add(TopoDS.Shell_s(_exp.Current())); _exp.Next()
            _mk.Build()
            if _mk.IsDone():
                _fix = ShapeFix_Solid(_mk.Solid())
                _fix.Perform()
                _solid = Solid(_fix.Shape())
            else:
                _solid = _sweep_shell
                print('WARNING: Sweep3 sweep — all solid attempts failed, result is Shell')
        from OCP.TopAbs import TopAbs_SHELL as _TS_SHELL, TopAbs_SOLID as _TS_SOLID
        if hasattr(_solid, 'wrapped') and _solid.wrapped.ShapeType() != _TS_SOLID:
            try:
                from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid as _MkSol2
                _mk2 = _MkSol2()
                _exp2 = TopExp_Explorer(_solid.wrapped, _TS_SHELL)
                while _exp2.More(): _mk2.Add(TopoDS.Shell_s(_exp2.Current())); _exp2.Next()
                _mk2.Build()
                if _mk2.IsDone(): _solid = Solid(_mk2.Shape())
            except Exception as _coerce_err:
                print('WARNING: Sweep3 Shell→Solid coercion failed:', _coerce_err)
        add(_solid, mode=Mode.ADD)
    except Exception as _sweep_err:
        print('WARNING: Sweep3 sweep failed:', _sweep_err)
    _face = _face_sk_Sketch96_52
    _vec = Vector(-0.500249, 0.00022, 0.865882) * -0.25
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch95_53
    _vec = Vector(-0.410461, -0.337515, 0.847116) * 19.0
    _solid = Solid.extrude(_face, _vec)
    _result_Extrude63 = cut_solids(part.part, _solid)
    if _result_Extrude63 is not None: part.part = _result_Extrude63
    _face = _face_sk_Sketch97_54
    _vec = Vector(0.0, 0.0, -1.0) * -4.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch102_55
    _vec = Vector(0.0, 0.0, 1.0) * 4.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch103_56
    _vec = Vector(-0.118438, -0.990618, -0.068174) * 1.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch104_57
    _vec = Vector(0.093471, -0.994143, 0.054254) * 1.293
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch105_58
    _vec = Vector(0.174086, -0.979555, 0.100824) * 0.998
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch106_59
    _vec = Vector(0.0, 0.0, 1.0) * 3.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch107_60
    _vec = Vector(0.613469, -0.789719, 0.0) * 0.5
    _solid = Solid.extrude(_face, _vec)
    _result_Extrude74 = cut_solids(part.part, _solid)
    if _result_Extrude74 is not None: part.part = _result_Extrude74
    _face = _face_sk_Sketch108_61
    _vec = Vector(0.0, 0.0, -1.0) * 10.0
    _solid = Solid.extrude(_face, _vec)
    _result_Extrude75 = cut_solids(part.part, _solid)
    if _result_Extrude75 is not None: part.part = _result_Extrude75
    _face = _face_sk_Sketch109_62
    _vec = Vector(0.0, 0.0, 1.0) * 3.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch109_63
    _vec = Vector(0.0, 0.0, 1.0) * 3.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    _face = _face_sk_Sketch110_64
    _vec = Vector(-0.410461, -0.337515, 0.847116) * -4.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    pass

export_stl(part.part,  'b_features.stl')

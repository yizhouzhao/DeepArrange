import omni.kit.commands
from pxr import UsdGeom, Usd, Gf

def get_prim_bbox(stage, prim):
    """
    Get prim bounding box
    """
    purposes = [UsdGeom.Tokens.default_]
    bboxcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), purposes)
    bboxes = bboxcache.ComputeWorldBound(prim)
    # print("bboxes", bboxes)
    game_bboxes = [bboxes.ComputeAlignedRange().GetMin(),bboxes.ComputeAlignedRange().GetMax()]

    return game_bboxes

def import_asset_to_stage(stage, prim_path, asset_path, position = (0,0,0), rotation = (1, 0, 0, 0), scale = 1):
    asset_prim = stage.GetPrimAtPath(prim_path)
    if not asset_prim.IsValid():
        asset_prim = stage.DefinePrim(prim_path)

    print("asset_prim:", asset_path)
    success_bool = asset_prim.GetReferences().AddReference(asset_path)

    assert success_bool, f"Asset loading unsuccessful at {asset_path}"

    xform_mat = Gf.Matrix4d().SetScale(scale) *  \
            Gf.Matrix4d().SetRotate(Gf.Quatf(float(rotation[0]), float(rotation[1]), float(rotation[2]), float(rotation[3]))) * \
            Gf.Matrix4d().SetTranslate([float(position[0]), float(position[1]), float(position[2])])

    # move to correct position and rotation
    omni.kit.commands.execute(
        "TransformPrimCommand",
        path=asset_prim.GetPath(),
        new_transform_matrix=xform_mat,
    )

    return asset_prim

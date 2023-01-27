import omni.kit.commands
import omni.usd
from pxr import UsdGeom, Usd, Gf, UsdLux, Sdf

def add_scene_default(task_type = "Wall"):
    """
    add default prim '/World' and '/DistantLight' to scene
    """
    stage = omni.usd.get_context().get_stage()
    if not stage.GetDefaultPrim():
        rootPrim = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(rootPrim.GetPrim())

    light_prim = stage.GetPrimAtPath("/World/defaultLight")
    if not light_prim.IsValid():
        # Adds a light to the scene
        distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/defaultLight"))
        distantLight.CreateIntensityAttr(3000)

        # (0.98079, -0.19509, 0, 0) # (0.65328, 0.2706, 0.2706, 0.65328)
        if task_type == "Bookshelf":
            distantLight.AddOrientOp().Set(Gf.Quatf(0.79829, -0.60227, 0, 0))
        else:
            distantLight.AddOrientOp().Set(Gf.Quatf(0.96593, -0.25882, 0, 0))

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
    """
    Import asset to path
    """
    prim_path = omni.usd.get_stage_next_free_path(stage, prim_path, True)

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

def add_force_field():
    """
    Note: this is not available in the current version:
    https://docs.omniverse.nvidia.com/prod_extensions/prod_extensions/ext_physics.html#physx-short-flatcache-also-known-as-fabric-rename-in-next-release
    """
     # enable api
    manager = omni.kit.app.get_app().get_extension_manager()
    forcefields_api_was_enabled = manager.is_extension_enabled("omni.physx.forcefields")
    if not forcefields_api_was_enabled:
        manager.set_extension_enabled_immediate("omni.physx.forcefields", True)

    import omni.physx.scripts.physicsUtils as physicsUtils

    from pxr import Gf, Sdf, Usd
    from pxr import UsdGeom, UsdUtils, UsdPhysics
    from pxr import PhysxSchema, PhysicsSchemaTools, ForceFieldSchema

    stage = omni.usd.get_context().get_stage()

    forcePrim = stage.GetPrimAtPath("/World/forcefield")
    if not forcePrim.IsValid():
        # create an unit xform
        omni.kit.commands.execute(
                "CreatePrim",
                prim_path="/World/forcefield",
                prim_type="Xform",
                select_new_prim=False,
            )


    boxSpacing = 2
    boxPathName = "/World/forcefield/box"

    # Create the force field prim
    # forcefield_prim.GetTranslateOp().Set(Gf.Vec3f(0.0, 200.0, 0.0))
    forcePrim = stage.GetPrimAtPath("/World/forcefield")

    sphericalPrimApi = ForceFieldSchema.PhysxForceFieldSphericalAPI.Apply(forcePrim, "Explode") # Suck
    sphericalPrimApi.CreateConstantAttr(1e4)
    sphericalPrimApi.CreateLinearAttr(0.0)
    sphericalPrimApi.CreateInverseSquareAttr(0.0)
    sphericalPrimApi.CreateEnabledAttr(False)
    sphericalPrimApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
    sphericalPrimApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0))

    noisePrimApi = ForceFieldSchema.PhysxForceFieldNoiseAPI.Apply(forcePrim, "Shake")
    noisePrimApi.CreateDragAttr(1e4)
    noisePrimApi.CreateAmplitudeAttr(Gf.Vec3f(100.0, 100.0, 0))
    noisePrimApi.CreateFrequencyAttr(Gf.Vec3f(4.0))
    noisePrimApi.CreateEnabledAttr(False)
    noisePrimApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
    noisePrimApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0))

    windPrimApi = ForceFieldSchema.PhysxForceFieldWindAPI.Apply(forcePrim, "Wind")
    windPrimApi.CreateDragAttr(1e4)
    windPrimApi.CreateAverageSpeedAttr(10.0)
    windPrimApi.CreateSpeedVariationAttr(10.0)
    windPrimApi.CreateSpeedVariationFrequencyAttr(0.5)
    windPrimApi.CreateAverageDirectionAttr(Gf.Vec3f(0.0, 1.0, 0.0))
    windPrimApi.CreateDirectionVariationAttr(Gf.Vec3f(0.707, 0.0, 0.707))
    windPrimApi.CreateDirectionVariationFrequencyAttr(Gf.Vec3f(0.5, 0.0, 0.5))
    windPrimApi.CreateEnabledAttr(False)
    windPrimApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
    windPrimApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0))

    # Add the collection 
    collectionAPI = Usd.CollectionAPI.ApplyCollection(forcePrim, ForceFieldSchema.Tokens.forceFieldBodies)
    collectionAPI.CreateIncludesRel().AddTarget(stage.GetDefaultPrim().GetPath())

    # Boxes
    boxSize = Gf.Vec3f(10.0)
    boxPosition = Gf.Vec3f(0.0)
    m = 2

    for i in range(m):
        for j in range(m):
            boxPath = boxPathName + str(i) + str(j)
            boxPosition[0] = (i + 0.5 - (0.5 * m)) * boxSpacing * boxSize[0]
            boxPosition[2] = 0.5 * boxSize[1]
            boxPosition[1] = 200 + (j + 0.5 - (0.5 * m)) * boxSpacing * boxSize[2]
            boxPrim = physicsUtils.add_rigid_box(stage, boxPath, position=boxPosition, size=boxSize)
    
    # enable primAPi
    sphericalPrimApi.GetEnabledAttr().Set(False) 
    noisePrimApi.GetEnabledAttr().Set(False)
    windPrimApi.GetEnabledAttr().Set(True)


def get_bounding_box(prim_path: str):
    """
    Get the bounding box of a prim
    """
    stage = omni.usd.get_context().get_stage()

    purposes = [UsdGeom.Tokens.default_]
    bboxcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), purposes)
    prim = stage.GetPrimAtPath(prim_path)
    bboxes = bboxcache.ComputeWorldBound(prim)
    # print("bboxes", bboxes)
    game_bboxes = [bboxes.ComputeAlignedRange().GetMin(),bboxes.ComputeAlignedRange().GetMax()]
    
    return game_bboxes
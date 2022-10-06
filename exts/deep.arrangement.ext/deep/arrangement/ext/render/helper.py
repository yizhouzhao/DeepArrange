import omni.usd
from omni.kit.viewport.utility import get_active_viewport

from pxr import Gf

class RenderHelper():
    def __init__(self, resolution = (1024, 1024)) -> None:
        self.resolution = resolution

        # setup view port    
        self.viewport = get_active_viewport()
        self.viewport.resolution = resolution

    def add_camera(self, camera_path = "/World/Camera", position = (0, 0, 0), rotation = (1, 0, 0, 0), **kwargs):
        stage = omni.usd.get_context().get_stage()
        target_path = omni.usd.get_stage_next_free_path(stage, camera_path, True)
        omni.kit.commands.execute("CreatePrimWithDefaultXformCommand", prim_path=target_path, prim_type="Camera", create_default_xform=True)
       
        # move to correct position and rotation
        xform_mat = Gf.Matrix4d().SetScale([1,1,1]) *  \
                Gf.Matrix4d().SetRotate(Gf.Quatf(float(rotation[0]), float(rotation[1]), float(rotation[2]), float(rotation[3]))) * \
                Gf.Matrix4d().SetTranslate([float(position[0]), float(position[1]), float(position[2])])

        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=target_path,
            new_transform_matrix=xform_mat,
        )
        
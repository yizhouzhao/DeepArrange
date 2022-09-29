import omni.ext
import omni.ui as ui

from .params import EXTENSION_FOLDER_PATH


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class MyExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[deep.arrangement.ext] MyExtension startup")

        self._count = 0

        self._window = ui.Window("Deep Arrangement", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                label = ui.Label("")

                with ui.HStack():
                    ui.Button("Add scene", clicked_fn=self.add_scene)

    ################################ scene #########################################

    def add_scene(self):
        print("Add scene EXTENSION_FOLDER_PATH", EXTENSION_FOLDER_PATH)

    def on_shutdown(self):
        print("[deep.arrangement.ext] MyExtension shutdown")

import importlib
from horizon.rhc import task_factory
from typing import List


class PluginInterface:
    """
    A plugin requires a given number of virtual functions to be defined.
    """

    @staticmethod  # def register_task_plugin() -> None:
    def register_task_plugin(factory) -> None:
        """
        Initialize the plugin.
        """
        pass


def import_module(name: str) -> PluginInterface:
    return importlib.import_module(name)  # type: ignore


def load_plugins(plugins: List[str]) -> None:
    """
    Load all plugins in the given list.
    """
    for name in plugins:
        plugin = import_module(name)
        plugin.register_task_plugin(task_factory)

# PLUGIN_NAME = 'pluginTry'
# def iter_namespace(ns_pkg):
#     # Specifying the second argument (prefix) to iter_modules makes the
#     # returned name an absolute name instead of a relative one. This allows
#     # import_module to work without having to do additional modification to
#     # the name.
#     return pkgutil.iter_modules(ns_pkg.__path__, ns_pkg.__name__ + ".")
#
# discovered_plugins = {
#     name: importlib.import_module(name)
#     for finder, name, ispkg
#     in iter_namespace(horizon.rhc.plugins)
# }

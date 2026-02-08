import os
from setuptools import setup

package_name = "robocat_vision"


def _asset_data_files():
    data_files = []
    for root, _, files in os.walk("assets"):
        if not files:
            continue
        rel_root = os.path.relpath(root, "assets")
        install_dir = f"share/{package_name}/assets"
        if rel_root != ".":
            install_dir = f"{install_dir}/{rel_root.replace(os.sep, '/')}"
        file_paths = [os.path.join(root, f) for f in files]
        data_files.append((install_dir, file_paths))
    return data_files

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ] + _asset_data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robocat",
    maintainer_email="dev@example.com",
    description="Robocat vision stub and event router.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vision_event_node = robocat_vision.vision_event_node:main",
            "vision_node = robocat_vision.vision_node:main",
        ],
    },
)

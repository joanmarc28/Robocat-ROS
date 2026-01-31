from setuptools import setup

package_name = "robocat_vision"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
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
        ],
    },
)

from setuptools import setup

package_name = "robocat_behavior"

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
    description="Robocat behavior and mode manager nodes.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mode_manager_node = robocat_behavior.mode_manager_node:main",
        ],
    },
)

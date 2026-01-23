from setuptools import find_packages, setup

package_name = "robocat_video"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robocat-v2",
    maintainer_email="22conan12@gmail.com",
    description="WebRTC video streaming helpers",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "webrtc_streamer_node = robocat_video.webrtc_streamer_node:main",
            "webrtc_signaling_node = robocat_video.webrtc_signaling_node:main",
        ],
    },
)

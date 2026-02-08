from pathlib import Path
from setuptools import find_packages, setup

package_name = 'robocat_hw'
share_dir = Path('share') / package_name
assets_root = Path('assets')

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

if assets_root.exists():
    for path in assets_root.rglob('*'):
        if not path.is_file():
            continue
        relative_dir = path.parent.as_posix()
        data_files.append((str(share_dir / relative_dir), [str(path)]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robocat-v2',
    maintainer_email='22conan12@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pi_status_node = robocat_hw.pi_status_node:main',
            'sensors_node = robocat_hw.sensors_node:main',
            'oled_message_node = robocat_hw.oled_message_node:main',
            'speaker_node = robocat_hw.speaker_node:main',
            'mic_node = robocat_hw.mic_node:main',
            'system_init_node = robocat_hw.system_init_node:main',
            'web_telemetry_node = robocat_hw.web_telemetry_node:main',
            'ws_command_node = robocat_hw.ws_command_node:main',
        ],
    },
)

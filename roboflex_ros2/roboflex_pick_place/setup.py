from glob import glob
import os

from setuptools import setup


package_name = "roboflex_pick_place"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml", "README.md", "task_list.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml") + glob("config/*.json"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="BrainSwarm",
    maintainer_email="brainswarms@gmail.com",
    description="External camera pick and place stack for RoboFlex.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scene_integration_node = roboflex_pick_place.scene_integration_node:main",
            "object_segmentation_node = roboflex_pick_place.object_segmentation_node:main",
        ],
    },
)

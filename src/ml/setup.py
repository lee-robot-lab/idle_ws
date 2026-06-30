from glob import glob

from setuptools import find_packages, setup

package_name = "ml"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["tests", "tests.*"]),
    py_modules=["detect_live", "ml_paths", "slot_tracker"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "requirements.txt"]),
        ("share/" + package_name + "/checkpoints", glob("checkpoints/*.md")),
        ("share/" + package_name + "/checkpoints/stage1_v2", glob("checkpoints/stage1_v2/*")),
        ("share/" + package_name + "/checkpoints/color_net_v2", glob("checkpoints/color_net_v2/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="eomyunbeen",
    maintainer_email="eomyunbeen@todo.local",
    description="Pure Python/PyTorch vision models for block detection, color grounding, and relation scoring.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
)

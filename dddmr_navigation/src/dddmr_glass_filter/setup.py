from glob import glob
from setuptools import find_packages, setup


package_name = "dddmr_glass_filter"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    python_requires=">=3.8",
    zip_safe=True,
    maintainer="DDDMR contributors",
    maintainer_email="frasherfrasher01@gmail.com",
    description="Known-glass-plane point cloud filtering and XT16 bag analysis.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "analyze_glass_bag = dddmr_glass_filter.analyze_bag:main",
            "export_odom_cloud = dddmr_glass_filter.export_odom_cloud:main",
            "fit_glass_plane = dddmr_glass_filter.fit_plane:main",
            "glass_plane_filter = dddmr_glass_filter.node:main",
        ],
    },
)

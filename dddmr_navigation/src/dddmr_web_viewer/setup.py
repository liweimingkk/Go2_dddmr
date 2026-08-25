from glob import glob
from setuptools import find_packages, setup


package_name = "dddmr_web_viewer"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md", "THIRD_PARTY_NOTICES.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/web", glob("web/*.*")),
        (f"share/{package_name}/web/vendor", glob("web/vendor/*.*")),
        (f"share/{package_name}/web/vendor/jsm/controls", glob("web/vendor/jsm/controls/*.*")),
    ],
    install_requires=["setuptools"],
    python_requires=">=3.8",
    zip_safe=True,
    maintainer="DDDMR contributors",
    maintainer_email="frasherfrasher01@gmail.com",
    description="Three.js web visualization with guarded DDDMR navigation actions.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "web_navigation_bridge = dddmr_web_viewer.web_navigation_bridge:main",
            "web_server = dddmr_web_viewer.web_server:main",
        ],
    },
)

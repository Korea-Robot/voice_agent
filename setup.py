from setuptools import find_packages, setup
from glob import glob
import os

package_name = "voice_agent"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jinwon Kim",
    maintainer_email="jwkim@krm.co.kr",
    description="Voice agent package for Vision60 robot",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "test_microphone = voice_agent.test_microphone:main",
            "voice_ui = voice_agent.ui:main",
            "voice_to_action = voice_agent.voice_to_action:main",
        ],
    },
)

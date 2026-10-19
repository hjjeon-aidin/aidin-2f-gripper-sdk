from setuptools import find_packages, setup

package_name = "aidin_gripper_examples"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AIDIN Robotics Firmware Team",
    maintainer_email="firmware@aidin.example",
    description="Python CLI examples for the AIDIN gripper ROS2 driver.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "state_listener  = aidin_gripper_examples.state_listener:main",
            "tactile_listener = aidin_gripper_examples.tactile_listener:main",
            "open_close_loop = aidin_gripper_examples.open_close_loop:main",
            "interactive_cli = aidin_gripper_examples.interactive_cli:main",
        ],
    },
)

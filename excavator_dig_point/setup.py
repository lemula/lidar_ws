from setuptools import find_packages, setup


package_name = "excavator_dig_point"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/dig_point.yaml"]),
        ("share/" + package_name + "/launch", ["launch/dig_point.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maintainer",
    maintainer_email="maintainer@example.com",
    description="Digging-point selector interface scaffold.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "dig_point_scaffold = excavator_dig_point.dig_point_scaffold:main",
        ],
    },
)


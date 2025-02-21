from setuptools import find_packages, setup

package_name = "dynaarm_extensions"

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
    maintainer="root",
    maintainer_email="timo.schwarzer@gmail.com",
    description="DynaArm Extensions - E-Stop and other utilities",
    license="TODO: License declaration",
    tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "e_stop_node = dynaarm_extensions.e_stop.e_stop_node:main",
        ],
    },
)

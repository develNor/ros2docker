from setuptools import setup


package_name = "e2e_std_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros2docker",
    maintainer_email="ros2docker@example.invalid",
    description="ros2docker E2E package that uses standard ROS messages.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["std_probe = e2e_std_pkg.std_probe:main"]},
)

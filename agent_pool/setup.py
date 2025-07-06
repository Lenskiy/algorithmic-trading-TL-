from setuptools import find_packages, setup

package_name = "agent_pool"

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
    maintainer_email="susanxu@ehu.edu.cn",
    description="Agent pool in ROS2",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ag1 = agent_pool.agents.RSI:main",
            "ag2 = agent_pool.agents.MACD:main",
            "ag3 = agent_pool.agents.BollingerBands:main",
            "ap = agent_pool.ap:main",
        ],
    },
)

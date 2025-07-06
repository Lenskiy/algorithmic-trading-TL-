from setuptools import find_packages, setup

package_name = "backtesting"

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
    maintainer_email="u7760022@anu.edu.au",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fake_dp = backtesting.fake.fake_dp:main",
            "fake_gtw = backtesting.fake.fake_gtw:main",
            "fake_ap = backtesting.fake.fake_ap:main",
            "fake_ag = backtesting.fake.fake_ag:main",
            "btm = backtesting.btm:main",
            "bt1 = backtesting.bt_pool.bt1:main",
            "bt2 = backtesting.bt_pool.bt2:main",
            "bt3 = backtesting.bt_pool.bt3:main",
        ],
    },
)

"""Setup the ezrassor_joy_translator module."""
import setuptools
import glob


setuptools.setup(
    name="ezrassor_joy_translator",
    version="2.0.0",
    description="Translate joy messages for the EZRASSOR.",
    maintainer="EZRASSOR Team",
    maintainer_email="ez.rassor@gmail.com",
    license="MIT",
    keywords=["EZRASSOR", "ROS", "ISRU", "NASA", "Rover", "UCF", "Robotics"],
    classifiers=[
        "Intended Audience :: Education",
        "Intended Audience :: Science/Research",
        "Programming Language :: Python",
        "Topic :: Education",
        "Topic :: Scientific/Engineering :: Astronomy",
        "Topic :: Scientific/Engineering :: Physics",
    ],
    packages=["ezrassor_joy_translator"],
    package_dir={"": "source"},
    install_requires=["setuptools"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resources/ezrassor_joy_translator"],
        ),
        ("share/ezrassor_joy_translator", ["package.xml"]),
        ("share/ezrassor_joy_translator/launch", glob.glob("launch/*")),
        ("share/ezrassor_joy_translator/config", glob.glob("config/*")),
    ],
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joy_translator = ezrassor_joy_translator.__main__:main",
        ],
    },
)

from setuptools import setup


setup(
    name="sonar",
    version="0.1",
    description="Wrapper for brping",
    author="Inspiration Robotics",
    py_modules=["sonar"],
    install_requires=["bluerobotics-ping"],
)

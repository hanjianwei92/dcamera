#!/usr/bin/env python
from setuptools import find_packages, setup


setup(
    name="dcamera",
    version="0.4",
    author="Han",
    url="10.0.8.172",
    description="dcamera interface (realsense mehmind)",
    packages=find_packages(exclude=("configs", "tests", "data", "scripts", "visualization")),
    data_files=[('dcamera/realsense_advance_mode_jsons',
                 ['dcamera/realsense_advance_mode_jsons/HighResHighAccuracyPreset.json',
                  'dcamera/realsense_advance_mode_jsons/HighResHighAccuracyPreset.json'])]
)

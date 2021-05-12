#!/usr/bin/env python
from setuptools import find_packages, setup


setup(
    name="dcamera",
    version="0.2",
    author="JarintotionDin",
    url="github.com",
    description="dcamera interface",
    packages=find_packages(exclude=("configs", "tests", "data", "scripts", "visualization")),
    data_files=[('dcamera/realsense_advance_mode_jsons',
                 ['dcamera/realsense_advance_mode_jsons/HighResHighAccuracyPreset.json',
                  'dcamera/realsense_advance_mode_jsons/HighResHighAccuracyPreset.json'])]
)

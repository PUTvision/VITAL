from glob import glob
import os
from setuptools import setup

package_name = 'visual_landing_provider'
version = '0.0.1'

if int(os.environ['ROS_VERSION']) == 1:
    setup(
        name=package_name,
        version=version,
        packages=[package_name],
        data_files=[
            ('share/' + package_name, ['config.yaml']),
            ('share/' + package_name + '/data', glob('./data/*.onnx'))
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='bartoszptak',
        maintainer_email='vision@put.poznan.pl',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
    )
else:
    submodules = [
        'visual_landing_provider/vital',
        'visual_landing_provider/vital/nn',
        'visual_landing_provider/vital/utils',
    ]

    setup(
        name=package_name,
        version=version,
        packages=[package_name] + submodules,
        data_files=[
            ('share/' + package_name, ['package.xml']),
            ('share/' + package_name, ['launch/visual_infer.launch.py']),
            ('share/' + package_name, ['config.yaml']),
            ('share/' + package_name + '/data', glob('./data/*.onnx'))
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='bartoszptak',
        maintainer_email='vision@put.poznan.pl',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'visual_infer_ros2 = visual_landing_provider.visual_infer_ros2:main' 
            ],
        },
    )

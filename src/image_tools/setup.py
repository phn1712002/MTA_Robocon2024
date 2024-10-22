from setuptools import find_packages, setup

package_name = 'image_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='phn1712002@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'switchcam = image_tools.switch_cam:main',
            'cam2image = image_tools.cam_image:main',
            'showyolo = image_tools.show_yolo:main',
        ],
    },
)

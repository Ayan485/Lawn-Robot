from setuptools import find_packages, setup

package_name = 'multinodal_lawnbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ayan48508@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['imgPub = multinodal_lawnbot.imgPub:main', 'graySub = multinodal_lawnbot.graySub:main', 'cannyEdge = multinodal_lawnbot.cannyEdge:main'
        ],
    },
)

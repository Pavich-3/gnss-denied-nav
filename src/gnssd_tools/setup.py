from setuptools import setup

package_name = 'gnssd_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pavlo Baranchuk',
    maintainer_email='pavlo.baranchuk@goreel.tech',
    description='Project nodes and evaluation scripts',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)

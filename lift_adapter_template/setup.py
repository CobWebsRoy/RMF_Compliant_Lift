from setuptools import setup

package_name = 'lift_adapter_template'

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
    maintainer='',
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lift_adapter_template = lift_adapter_template.lift_adapter_template:main',
            'qr_decoder = lift_adapter_template.qr_decoder:main'
        ],
    },
)

from setuptools import setup
import glob

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', ['launch/view_robot.launch.py']),
        ('share/' + package_name + '/urdf/',   glob.glob('urdf/*.urdf')),

        ('share/' + package_name + '/meshes/abb_irb6700_150_320/visual',  glob.glob('meshes/abb_irb6700_150_320/visual/*.glb')),
        ('share/' + package_name + '/meshes/abb_irb6700_150_320/collision',  glob.glob('meshes/abb_irb6700_150_320/collision/*.obj')),

        ('share/' + package_name + '/meshes/linear_axis/collision',  glob.glob('meshes/linear_axis/collision/*.obj')),
        ('share/' + package_name + '/meshes/linear_axis/visual',  glob.glob('meshes/linear_axis/visual/*.glb')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='abdofarhan75@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

from setuptools import setup
import os
from glob import glob

package_name = 'robot_arm_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 파일 추가
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # URDF 파일 추가
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # Config 파일 추가
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # Meshes 폴더가 있다면 추가 (향후 확장성)
        # (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Robot arm URDF description and visualization',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # URDF 패키지에는 실행노드가 필요 없음
        ],
    },
)

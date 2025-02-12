from setuptools import find_packages, setup
import os
import glob

package_name = 'voiceRecognition'
model_src_dir = os.path.join('vosk-model-small-en-us-0.15')
model_dest_dir = os.path.join('share', package_name, 'vosk-model-small-en-us-0.15')

model_files = [(os.path.join(model_dest_dir, os.path.relpath(f, model_src_dir)), [f])
               for f in glob.glob(os.path.join(model_src_dir, '**', '*'), recursive=True) if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + model_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='singh',
    maintainer_email='ashutosh19082003@gmail.com',
    description='description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voiceControl = voiceRecognition.voice_command:main',
        ],
    },
)

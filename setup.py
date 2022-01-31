from setuptools import setup
import subprocess as sp
from pathlib import Path
import sys

package_name = 'canros'

# Run package generation at build time
script_path = Path(__file__).parent.absolute() / "scripts/generate.py"
rc = sp.run(["python3", script_path])
if rc.returncode != 0:
    # sp.run() doesn't capture stdout/stderr by default, so no need to print here
    sys.exit(rc.returncode)

setup(
 name=package_name,
 version='0.1.0',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
   ],
 install_requires=['setuptools', 'pyuavcan-v0'],
 zip_safe=True,
 maintainer='James Stewart',
 maintainer_email='james.stewy@gmail.com',
 description='UAVCAN to ROS interface.',
 license='BSD',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'canros_server = canros.server:main'
     ],
   },
)

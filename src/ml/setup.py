from setuptools import find_packages, setup

setup(
    name='ml',
    version='0.0.1',
    packages=find_packages(exclude=['test', 'tests', '__pycache__']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ml']),
        ('share/ml', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='su',
    maintainer_email='lsu031111@hanyang.ac.kr',
    description='Stage1/2/4 ML models — slot encoder, color net, relation grounding.',
    license='Apache-2.0',
)

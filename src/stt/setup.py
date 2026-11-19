from setuptools import find_packages, setup

setup(
    name='stt',
    version='0.0.1',
    packages=find_packages(exclude=['test', 'tests', '__pycache__']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/stt']),
        ('share/stt', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='su',
    maintainer_email='lsu031111@hanyang.ac.kr',
    description='STT pipeline (rule/Whisper/LLM) → semantic pick-place step.',
    license='Apache-2.0',
)

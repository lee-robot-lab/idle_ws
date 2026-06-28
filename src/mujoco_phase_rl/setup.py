from setuptools import find_packages, setup

package_name = 'mujoco_phase_rl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
        ('share/' + package_name, ['CLI_README.ko.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='su',
    maintainer_email='lsu031111@hanyang.ac.kr',
    description='Phase-conditioned MuJoCo RL prototype for IDLE pick-place tasks.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'random_rollout = mujoco_phase_rl.policies.random_rollout:main',
            'scripted_rollout = mujoco_phase_rl.policies.scripted_rollout:main',
            'train_ppo = mujoco_phase_rl.policies.train_ppo:main',
            'train_sac = mujoco_phase_rl.policies.train_sac:main',
            'rollout_policy = mujoco_phase_rl.policies.rollout_policy:main',
            'sim_phase_diagnostics = mujoco_phase_rl.policies.sim_phase_diagnostics:main',
            'evaluate_policy = mujoco_phase_rl.policies.evaluate_policy:main',
            'collect_vision_dataset = mujoco_phase_rl.policies.collect_vision_dataset:main',
            'train_vision_estimator = mujoco_phase_rl.policies.train_vision_estimator:main',
            'evaluate_vision_estimator = mujoco_phase_rl.policies.evaluate_vision_estimator:main',
            'predict_vision_image = mujoco_phase_rl.policies.predict_vision_image:main',
            'summarize_vision_dataset = mujoco_phase_rl.policies.summarize_vision_dataset:main',
            'camera_smoke = mujoco_phase_rl.policies.camera_smoke:main',
            'vision_pose_adapter = mujoco_phase_rl.bridges.vision_pose_adapter:main',
            'real_sensor_check = mujoco_phase_rl.bridges.real_sensor_check:main',
            'real_phase_diagnostics = mujoco_phase_rl.bridges.real_phase_diagnostics:main',
            'real_episode_recorder = mujoco_phase_rl.bridges.real_episode_recorder:main',
            'annotate_real_fusion_run = mujoco_phase_rl.bridges.annotate_real_fusion_run:main',
            'summarize_real_fusion_dataset = mujoco_phase_rl.bridges.summarize_real_fusion_dataset:main',
            'real_action_bridge = mujoco_phase_rl.bridges.real_action_bridge:main',
            'sim_sensor_bridge = mujoco_phase_rl.bridges.sim_sensor_bridge:main',
        ],
    },
)

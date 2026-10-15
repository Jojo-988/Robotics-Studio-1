"""Verify launch policies without starting Gazebo or altering the caller's environment."""
import importlib.util
import os
from pathlib import Path
import subprocess
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from patrol.preflight import check_environment


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('patrol_launch', ROOT / 'launch/41068_coverage_patrol.launch.py')
launch_file = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_file)


class StartupTests(unittest.TestCase):
    def actions(self, **options):
        context = LaunchContext()
        context.launch_configurations.update(
            mode='standalone', start_sim='auto', use_sim_time='true', partition='auto',
            rendering='auto', world='large_demo', gz_gui='false', params_file='unused',
            repeat='false', altitude='15.0')
        context.launch_configurations.update(spawn_x='0.0', spawn_y='-8.0', spawn_z='0.8')
        context.launch_configurations.update(options)
        with patch.object(launch_file, 'check_environment', return_value=([], [])):
            actions = launch_file._launch(context)
        for action in actions:
            if isinstance(action, SetEnvironmentVariable):
                action.execute(context)
        return dict(context.environment), actions

    def test_standalone_isolates_gazebo_even_if_old_partition_is_inherited(self):
        with patch.dict(os.environ, IGN_PARTITION='old', GZ_PARTITION='old'):
            first, actions = self.actions()
            second, _ = self.actions()
        self.assertNotEqual(first['IGN_PARTITION'], 'old')
        self.assertNotEqual(first['IGN_PARTITION'], second['IGN_PARTITION'])
        self.assertEqual(first['IGN_PARTITION'], first['GZ_PARTITION'])
        self.assertEqual(sum(isinstance(a, IncludeLaunchDescription) for a in actions), 1)

    def test_attach_preserves_existing_session_and_starts_no_simulator(self):
        with patch.dict(os.environ, IGN_PARTITION='team', GZ_PARTITION='team', LIBGL_ALWAYS_SOFTWARE='0'):
            env, actions = self.actions(mode='attach', rendering='software')
            _, legacy = self.actions(start_sim='false')
        self.assertEqual(env['IGN_PARTITION'], 'team')
        self.assertEqual(env['LIBGL_ALWAYS_SOFTWARE'], '0')
        for collection in (actions, legacy):
            self.assertFalse(any(isinstance(a, IncludeLaunchDescription) for a in collection))

    def test_explicit_rendering_and_partition(self):
        env, _ = self.actions(partition='test_session', rendering='software')
        self.assertEqual(env['LIBGL_ALWAYS_SOFTWARE'], '1')
        self.assertEqual(env['IGN_PARTITION'], 'test_session')
        env, _ = self.actions(rendering='hardware')
        self.assertEqual(env['LIBGL_ALWAYS_SOFTWARE'], '0')

    def test_simulation_rejects_wall_time(self):
        with self.assertRaisesRegex(ValueError, 'use_sim_time'):
            self.actions(use_sim_time='false')

    def test_takeoff_column_is_forwarded_to_the_robot_spawner(self):
        with patch.object(launch_file, 'IncludeLaunchDescription') as include:
            self.actions(spawn_x='1.0', spawn_y='-9.0')
        args = dict(include.call_args.kwargs['launch_arguments'])
        self.assertEqual((args['parrot_x'], args['parrot_y'], args['parrot_z']), ('1.0', '-9.0', '0.8'))
        with self.assertRaises(ValueError):
            self.actions(spawn_x='nan')

    def test_wrong_workspace_and_invalid_domain_fail_preflight(self):
        with patch.dict(os.environ, ROS_DOMAIN_ID='invalid'):
            errors, _ = check_environment('attach', '/unrelated/workspace')
        self.assertTrue(any('ROS_DOMAIN_ID' in item for item in errors))
        self.assertTrue(any('shadows' in item for item in errors))

    def test_shell_check_preserves_team_network_settings(self):
        env = dict(os.environ, ROS_DOMAIN_ID='71', ROS_LOCALHOST_ONLY='0')
        env.pop('PATROL_ROS_DOMAIN_ID', None)
        env.pop('PATROL_WORKSPACE', None)
        result = subprocess.run(['bash', str(ROOT / 'scripts/start_coverage_patrol.sh'),
                                 '--check', 'mode:=attach'], env=env, text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=15)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('ROS_DOMAIN_ID=71; ROS_LOCALHOST_ONLY=0', result.stdout)
        self.assertIn('Mode: attach', result.stdout)


if __name__ == '__main__':
    unittest.main()

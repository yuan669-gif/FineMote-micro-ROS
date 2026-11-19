import pathlib
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/urdf_codegen'))
from devices import device_graph, load_controllers, ros_source
from generate import Generator

EXAMPLE = ROOT / 'Interface/Examples/URDF'


class DeviceDescription(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(EXAMPLE / 'pov_simple.urdf').getroot()
        self.config = load_controllers(EXAMPLE / 'pov_controllers.yaml')

    def graph(self):
        return device_graph(self.root, self.config)

    def test_communication(self):
        _, manifest = self.graph()
        code = ros_source(manifest)
        self.assertEqual(code.count('JointPublisher pub_'), 8)
        self.assertEqual(manifest['ros']['agents'], 10)
        self.assertEqual(manifest['ros']['executor_handles'], 1)
        self.assertEqual(manifest['ros']['subscriptions'], ['chassis/cmd_vel'])
        self.assertIn('chassis/Twist', manifest['ros']['publishers'])
        manifest['joints']['extra'] = manifest['joints']['wheel_joint_FR']
        with self.assertRaisesRegex(ValueError, 'capacity'):
            ros_source(manifest)

    def test_reference_graph(self):
        block, manifest = self.graph()
        self.root.append(block)
        graph = Generator(self.root, None)
        self.assertEqual(len(graph.objects), 17)
        seen = set()
        for name in graph.order:
            self.assertTrue(graph.deps[name] <= seen)
            seen.add(name)
        self.assertEqual(graph.order[-1], 'chassis')
        joints = manifest['joints']
        self.assertEqual(len({item['controller'] for item in joints.values()}), 8)
        modules = graph.objects['chassis'].findall("arg[@name='configs']/aggregate")
        for number, corner in enumerate(('FR', 'FL', 'BL', 'BR'), start=1):
            steer, drive = joints['steer_joint_' + corner], joints['wheel_joint_' + corner]
            for item, addr, period, kind in ((steer, number, 20, 'Amplifier'), (drive, 0x140 + number, 1, 'PID')):
                motor = graph.objects[item['motor']]
                self.assertEqual(graph.objects[item['controller']].get('type'), kind)
                self.assertEqual(motor.find("arg[@name='_controller']").get('ref'), item['controller'])
                self.assertEqual(motor.find("arg[@name='addr']").get('value'), hex(addr))
                self.assertEqual(motor.find("arg[@name='divisionFactor']").get('value'), str(period))
                self.assertEqual(motor.find(".//field[@name='multiTurnSamePosition']").get('value'), 'true')
            module = modules[number - 1]
            self.assertEqual(module.find("field[@name='driveMotor']").get('ref'), drive['motor'])
            self.assertEqual(module.find("field[@name='steerMotor']").get('ref'), steer['motor'])
            self.assertEqual(module.find("field[@name='zeroPosition']").get('value'),
                             '180.0f' if corner in ('FR', 'BR') else '0.0f')

    def test_unordered_declarations(self):
        expected, _ = self.graph()
        control = self.root.find('ros2_control')
        control[:] = reversed(control[:])
        self.config['controllers'].reverse()
        actual, _ = self.graph()
        self.assertEqual(ET.tostring(actual), ET.tostring(expected))

    def test_controller_errors(self):
        controller = self.config['controllers'][0]['controller']
        for value in (0, 256, 1.5, True):
            controller['division_factor'] = value
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'division_factor'):
                self.graph()
        controller['division_factor'] = 1
        controller['joints'].append(controller['joints'][0])
        with self.assertRaisesRegex(ValueError, 'duplicate controller'):
            self.graph()
        controller['joints'][-1] = 'missing'
        with self.assertRaisesRegex(ValueError, 'mismatch'):
            self.graph()

    def test_bad_hardware(self):
        for path, key, value, error in (
            ('motor', 'type', '9999', 'combination'),
            ('command_interface', 'name', 'position', 'combination'),
            ('bus', 'output', 'Torque', 'expected keys'),
            ("bus/param[@name='id']", None, '0x142', 'duplicate bus address'),
            ("bus/param[@name='port']", None, '3', 'integer'),
        ):
            node = self.root.find("ros2_control/joint[@name='wheel_joint_FR']/" + path)
            saved = dict(node.attrib), node.text
            if key:
                node.set(key, value)
            else:
                node.text = value
            with self.subTest(path=path), self.assertRaisesRegex(ValueError, error):
                self.graph()
            node.attrib, node.text = saved

    def test_bad_topology(self):
        self.root.find("joint[@name='wheel_joint_FR']/parent").set('link', 'base_link')
        with self.assertRaisesRegex(ValueError, 'base_link -> steer -> drive'):
            self.graph()

    def test_duplicate_yaml_key(self):
        with tempfile.TemporaryDirectory() as folder:
            path = pathlib.Path(folder) / 'bad.yaml'
            path.write_text('version: 1\nversion: 2\n', encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'duplicate YAML key'):
                load_controllers(path)


if __name__ == '__main__':
    unittest.main()

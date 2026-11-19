"""Lower FineMote device descriptions to the named construction graph used by the AST backend."""
import math
import re
import xml.etree.ElementTree as ET


def expect(condition, message):
    if not condition:
        raise ValueError(message)


def load_controllers(path):
    try:
        import yaml
    except ImportError as error:
        raise ValueError('install PyYAML in the Python interpreter used by CMake') from error

    class UniqueLoader(yaml.SafeLoader):
        def construct_mapping(self, node, deep=False):
            result = super().construct_mapping(node, deep)
            expect(len(result) == len(node.value), f'{path}:{node.start_mark.line + 1}: duplicate YAML key')
            return result

    try:
        return yaml.load(path.read_text(encoding='utf-8'), Loader=UniqueLoader)
    except yaml.YAMLError as error:
        raise ValueError(f'{path}: {error}') from error


def keys(mapping, names, context):
    expect(isinstance(mapping, dict) and set(mapping) == set(names.split()),
           f'{context}: expected keys [{names}], got {list(mapping) if isinstance(mapping, dict) else mapping!r}')


def integer(value, low, high, context):
    expect(type(value) is int and low <= value <= high, f'{context}: expected integer {low}..{high}, got {value!r}')
    return value


def scalar(value, context):
    try:
        number = float(value)
    except (ValueError, TypeError, OverflowError) as error:
        raise ValueError(f'{context}: invalid number {value!r}') from error
    expect(math.isfinite(number) and abs(number) <= 3.4028234663852886e38 and
           (number == 0 or abs(number) >= 1.401298464324817e-45), f'{context}: invalid float {value!r}')
    return number


def controllers_by_joint(config):
    keys(config, 'version controllers', 'controller YAML')
    integer(config['version'], 1, 1, 'version')
    groups = config['controllers']
    expect(isinstance(groups, list) and groups, 'controllers: expected nonempty list')
    bindings = {}
    for entry in groups:
        keys(entry, 'controller', 'controllers entry')
        controller = entry['controller']
        keys(controller, 'type joints division_factor parameters', 'controller')
        kind, params = controller['type'], controller['parameters']
        expect(kind in ('PID', 'Amplifier'), f'unsupported controller {kind!r}')
        integer(controller['division_factor'], 1, 255, 'division_factor')
        keys(params, 'kp ki kd' if kind == 'PID' else 'gain', kind)
        if kind == 'PID':
            for name, value in params.items():
                expect(type(value) in (int, float), f'PID.{name}: expected numeric gain')
                scalar(value, name)
        else:
            integer(params['gain'], 0, 0xffffffff, 'Amplifier.gain')
        joints = controller['joints']
        expect(isinstance(joints, list) and joints, f'{kind}: expected nonempty joints list')
        for joint in joints:
            expect(isinstance(joint, str) and joint, 'controller: invalid joint name')
            expect(joint not in bindings, f'{joint}: duplicate controller binding')
            bindings[joint] = controller
    return bindings


def indexed(nodes, context):
    result = {}
    for node in nodes:
        name = node.get('name')
        expect(name and name not in result, f'{context}: missing or duplicate name {name!r}')
        result[name] = node
    return result


def shape(node, attrs='', children=''):
    keys(node.attrib, attrs, node.tag)
    expect(not (node.text or '').strip() or node.tag == 'param', f'{node.tag}: unexpected text')
    for child in node:
        expect(child.tag in children.split() and not (child.tail or '').strip(), f'{node.tag}: unexpected {child.tag} or text')


def single(node, tag):
    matches = node.findall(tag)
    expect(len(matches) == 1, f'{node.get("name", node.tag)}: expected one {tag}')
    return matches[0]


def named_params(node):
    result = {}
    for name, param in indexed(node.findall('param'), node.tag + ' parameters').items():
        shape(param, 'name')
        result[name] = (param.text or '').strip()
    return result


def interfaces(node, tag, allowed):
    items = indexed(node.findall(tag), node.get('name') + '/' + tag)
    for name, item in items.items():
        shape(item, 'name')
        expect(name in allowed, f'{node.get("name")}: unsupported {tag} {name}')
    return list(items)


def bus_number(text, low, high, context):
    expect(re.fullmatch(r'(0[xX][0-9a-fA-F]+|[0-9]+)', text), f'{context}: invalid bus number {text!r}')
    return integer(int(text, 16 if text.lower().startswith('0x') else 10), low, high, context)


ADAPTERS = {
    ('4010', 'can', 'velocity', 'PID'): ('Motor4010', 'busID', 'Torque', 'Speed'),
    ('4315', 'rs485', 'position', 'Amplifier'): ('Motor4315', 'BusID', 'Position', 'Position'),
}


def emit_motor(block, joint, controller, index, endpoints):
    name = joint.get('name')
    shape(joint, 'name', 'command_interface state_interface motor bus')
    motor, bus = single(joint, 'motor'), single(joint, 'bus')
    shape(motor, 'type')
    shape(bus, 'type', 'param')
    commands = interfaces(joint, 'command_interface', ('velocity', 'position'))
    states = interfaces(joint, 'state_interface', ('velocity', 'position'))
    expect(len(commands) == 1, f'{name}: expected one command interface')
    model, transport, kind = motor.get('type'), bus.get('type'), controller['type']
    combination = (model, transport, commands[0], kind)
    expect(combination in ADAPTERS, f'{name}: unsupported device combination {combination}')
    driver, port_field, output, target = ADAPTERS[combination]
    params = named_params(bus)
    address_key = 'id' if transport == 'can' else 'address'
    keys(params, 'port ' + address_key, f'{name}/{transport}')
    port = bus_number(params['port'], 1, 2, f'{name}/port')
    address = bus_number(params[address_key], 0, 0x7ff if transport == 'can' else 255, f'{name}/{address_key}')
    endpoint = (transport, port, address)
    expect(endpoint not in endpoints, f'{name}: duplicate bus address {endpoint}')
    endpoints.add(endpoint)
    controller_id, motor_id = f'controller_{index}', f'motor_{index}'
    algorithm = ET.SubElement(block, 'object', id=controller_id, type=kind)
    gains = controller['parameters']
    if kind == 'PID':
        aggregate = ET.SubElement(ET.SubElement(algorithm, 'arg', name='params'), 'aggregate')
        for key, value in gains.items():
            ET.SubElement(aggregate, 'field', name=key, value=repr(float(value)) + 'f')
    else:
        ET.SubElement(algorithm, 'template', name='K', value=str(gains['gain']))
    device = ET.SubElement(block, 'object', id=motor_id, type=driver)
    ET.SubElement(device, 'template', name=port_field, value=str(port))
    aggregate = ET.SubElement(ET.SubElement(device, 'arg', name='params'), 'aggregate')
    for field, mode in [('ctrlType', output), ('targetType', target)]:
        ET.SubElement(aggregate, 'field', name=field, value='Motor_Ctrl_Type_e::' + mode)
    ET.SubElement(aggregate, 'field', name='multiTurnSamePosition', value='true')
    ET.SubElement(device, 'arg', name='_controller', ref=controller_id)
    ET.SubElement(device, 'arg', name='addr', value=hex(address))
    ET.SubElement(device, 'arg', name='divisionFactor', value=str(controller['division_factor']))
    return dict(motor=motor_id, controller=controller_id, model=model, command=commands[0], states=states,
                bus=transport, port=port, address=address, division_factor=controller['division_factor'])


def emit_chassis(block, chassis, mechanical, motors, links):
    shape(chassis, 'name type', 'command_interface state_interface param odometry module')
    odom = single(chassis, 'odometry')
    shape(odom, 'type')
    expect(chassis.get('name') and chassis.get('type') == 'pov' and odom.get('type') == 'planar',
           'expected named POV chassis with planar odometry')
    commands = interfaces(chassis, 'command_interface', ('velocity',))
    states = interfaces(chassis, 'state_interface', ('velocity',))
    params = named_params(chassis)
    keys(params, 'wheel_diameter', 'chassis parameters')
    diameter = scalar(params['wheel_diameter'], 'wheel_diameter')
    expect(diameter > 0, 'wheel_diameter must be positive')
    obj = ET.SubElement(block, 'object', id='chassis', factory='POV_ChassisBuilder')
    ET.SubElement(obj, 'template', name='OdomPolicy', type='PlanarOdom')
    ET.SubElement(obj, 'arg', name='_wheelDiameter', value=repr(diameter) + 'f')
    used, modules = set(), []
    for module in chassis.findall('module'):
        shape(module, 'steer drive', 'param')
        steer, drive = module.get('steer'), module.get('drive')
        for joint, command in ((steer, 'position'), (drive, 'velocity')):
            expect(joint in motors and motors[joint]['command'] == command, f'module: invalid {command} joint {joint}')
            expect(joint not in used, f'module: repeated joint {joint}')
            used.add(joint)
        steer_link = single(mechanical[steer], 'child').get('link')
        expect(steer_link in links and single(mechanical[steer], 'parent').get('link') == 'base_link' and
               single(mechanical[drive], 'parent').get('link') == steer_link, f'{steer}/{drive}: expected base_link -> steer -> drive')
        origin = mechanical[steer].find('origin')
        if origin is not None:
            rotation = [scalar(value, steer + '/rpy') for value in origin.get('rpy', '0 0 0').split()]
            expect(rotation == [0, 0, 0], f'{steer}: rotated steer frame not supported')
        xyz = (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()
        expect(len(xyz) == 3, f'{steer}: expected three origin coordinates')
        position = [scalar(value, steer + '/xyz') for value in xyz]
        params = named_params(module)
        expect(params.keys() <= {'zero_position'}, f'{steer}: unknown module parameter')
        zero = scalar(math.degrees(scalar(params.get('zero_position', 0), 'zero_position')), 'zeroPosition')
        config = ET.SubElement(ET.SubElement(obj, 'arg', name='configs'), 'aggregate', type='Swerve_t')
        for role, joint in (('steerMotor', steer), ('driveMotor', drive)):
            ET.SubElement(config, 'field', name=role, ref=motors[joint]['motor'], address='true')
        for field, value in [('lx', position[0]), ('ly', position[1]), ('zeroPosition', zero)]:
            ET.SubElement(config, 'field', name=field, value=repr(value) + 'f')
        modules.append(dict(steer=steer, drive=drive))
    expect(modules, 'chassis: expected at least one module')
    return dict(name=chassis.get('name'), modules=modules, commands=commands, states=states)


def ros_source(manifest):
    lines = ['#include "URDF/DeviceROS.hpp"', 'namespace {']
    publishers, subscriptions = [], []
    for name, binding in manifest['joints'].items():
        if not binding['states']:
            continue
        expect(re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', name), f'invalid ROS topic component: {name}')
        motor = binding['motor']
        topic = 'joints/' + name
        flags = ', '.join(str(field in binding['states']).lower() for field in ('position', 'velocity'))
        lines.extend([f'char name_{motor}[] = "{name}";',
                      f'finemote_urdf::JointPublisher pub_{motor}{{"{topic}", name_{motor}, {motor}, {flags}}};'])
        publishers.append(topic + '/JointState')
    chassis = manifest.get('chassis')
    if chassis:
        name = chassis['name']
        expect(re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', name), f'invalid ROS topic component: {name}')
        if chassis['states']:
            lines.append(f'RosPublisher pub_chassis{{"{name}", finemote_urdf::PlanarState(chassis)}};')
            publishers.append(name + '/Twist')
        if chassis['commands']:
            topic = name + '/cmd_vel'
            lines.append(f'finemote_urdf::PlanarSubscriber sub_chassis{{"{topic}", chassis}};')
            subscriptions.append(topic)
    count = len(publishers) + len(subscriptions)
    expect(count <= 10, f'micro-ROS requires {count} agents, capacity is 10')
    lines.extend([f'static_assert({count} <= MICROROS_MAX_AGENTS);',
                  f'static_assert({len(publishers)} <= RMW_UXRCE_MAX_PUBLISHERS);',
                  f'static_assert({len(subscriptions)} <= RMW_UXRCE_MAX_SUBSCRIPTIONS && '
                  f'{len(subscriptions)} <= MICROROS_MAX_HANDLES);', '}'])
    manifest['ros'] = dict(publishers=publishers, subscriptions=subscriptions, agents=count,
                           executor_handles=len(subscriptions))
    return '\n'.join(lines) + '\n'


def device_graph(root, config):
    expect(root.tag == 'robot' and not any(root.findall(tag) for tag in ('finemote', 'finemote_cpp', 'transmission')),
           'expected FineMote device URDF without legacy extensions')
    mechanical = indexed(root.findall('joint'), 'mechanical joints')
    links = indexed(root.findall('link'), 'links')
    control = single(root, 'ros2_control')
    shape(control, 'name type', 'joint chassis')
    expect(control.get('type') == 'system', 'expected ros2_control type=system')
    joints = indexed(control.findall('joint'), 'control joints')
    bindings = controllers_by_joint(config)
    expect(joints.keys() == bindings.keys(), f'controller/joint mismatch: {sorted(joints.keys() ^ bindings.keys())}')
    block = ET.Element('finemote_cpp')
    ET.SubElement(block, 'include', header='URDF/POVDevices.hpp')
    motors, endpoints = {}, set()
    for index, name in enumerate(sorted(joints)):
        expect(name in mechanical, f'unknown mechanical joint {name}')
        for tag in ('parent', 'child'):
            expect(single(mechanical[name], tag).get('link') in links, f'{name}: unknown {tag} link')
        motors[name] = emit_motor(block, joints[name], bindings[name], index, endpoints)
    manifest = {'joints': motors}
    if control.findall('chassis'):
        manifest['chassis'] = emit_chassis(block, single(control, 'chassis'), mechanical, motors, links)
    return block, manifest

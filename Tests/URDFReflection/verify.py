"""Integration acceptance tests for the explicit C++17 construction graph."""
import copy, json, os, pathlib, re, subprocess, sys, xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[2]
BUILD = pathlib.Path(os.environ.get('FINEMOTE_REFLECTION_BUILD_DIR', ROOT/'build/reflection-verification')).resolve()
BUILD.mkdir(parents=True,exist_ok=True)
CLANG = os.environ.get('FINEMOTE_AST_CLANG',r'D:\new_STM32\STM32\ARM\ARMCLANG\bin\armclang.exe')
GCCBIN = pathlib.Path(os.environ.get('FINEMOTE_GCC_BIN',r'C:\ST\STM32CubeCLT_1.18.0\GNU-tools-for-STM32\bin'))
os.environ['PATH'] = str(GCCBIN)+os.pathsep+os.environ['PATH']
os.environ['PYTHONUTF8']='1'
GEN = ROOT/'tools/urdf_codegen/generate.py'
URDF = ROOT/'Interface/Examples/URDF/pov_reflected.urdf'
LABELS = ['firmware','cxx17','manual_equivalence','reverse_order','field_order','missing_kd',
          'unknown_field','duplicate_field','extra_argument','wrong_type','invalid_bus',
          'missing_reference','cycle','specialization_const','constructor_default',
          'urdf_rebuild','unknown_model','extra_template_argument','shared_bus_id','task_registration']
results = {k:False for k in LABELS}
def command(name,args):
    p=subprocess.run([str(a) for a in args],cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    (BUILD/(name+'.log')).write_bytes(p.stdout)
    return p.returncode, p.stdout.decode('utf-8',errors='replace')
def record(name,ok):
    results[name]=bool(ok)
    print(name+': '+('PASS' if ok else 'FAIL'),flush=True)
def generate(name,tree):
    src=BUILD/(name+'.urdf'); tree.write(src,encoding='utf-8',xml_declaration=True)
    out=BUILD/(name+'.cpp')
    rc,log=command(name,[sys.executable,GEN,'--urdf',src,'--compile-db',BUILD/'compile_commands.json',
        '--clang',CLANG,'--output',out,'--metadata',BUILD/(name+'.json'),'--check'])
    return rc, out.read_text(encoding='utf-8') if out.exists() else '', log
def obj(tree,name): return tree.find("./finemote_cpp/object[@id='"+name+"']")
def pid(tree): return obj(tree,'wheel_BR').find("./arg[@name='params']/aggregate")
def main():
    if not GEN.exists() or not URDF.exists():
        print('AST generator/example absent: baseline = 0')
        return
    rc,_=command('configure',['cmake','--preset','ast-MC_Board-release','-S',ROOT,'-B',BUILD,
        '-DFINEMOTE_CONTROLLERS_YAML=',
        '-DCMAKE_BUILD_TYPE=Release','-DFINEMOTE_REFLECTED_URDF='+str(URDF),'-DFINEMOTE_AST_CLANG='+CLANG,
        '-DPython3_EXECUTABLE='+sys.executable])
    if rc: return
    rc,_=command('firmware',['cmake','--build',BUILD,'--parallel','6'])
    record('firmware',rc==0 and (BUILD/'MC_Board.elf').is_file())
    db=json.loads((BUILD/'compile_commands.json').read_text(encoding='utf-8'))
    cpp=[x['command'] for x in db if pathlib.Path(x['file']).suffix=='.cpp']
    record('cxx17',bool(cpp) and all(set(re.findall(r'-std=[^\s"]+', x)) == {'-std=c++17'} for x in cpp))
    tree=ET.parse(URDF)
    rc,source,_=generate('base',tree)
    norm=re.sub(r'\s+','',source)
    expected=['PIDwheel_BR{PID_Param_t{0.23f,0.008f,0.3f}}',
              'Motor4010<1>CBRMotor{Motor_Param_t{Motor_Ctrl_Type_e::Torque,Motor_Ctrl_Type_e::Speed},wheel_BR,0x144}',
              'Motor4315<1>SFRMotor{Motor_Param_t{Motor_Ctrl_Type_e::Position,Motor_Ctrl_Type_e::Position,true},steer_FR,0x01,20}',
              'POV_ChassisBuilder<PlanarOdom>(0.0483f,',
              'Swerve_t{&SFRMotor,&CFRMotor,0.1201125f,-0.12f,180}',
              'Swerve_t{&SBLMotor,&CBLMotor,-0.1201125f,0.12f}',
              'UARTBuffer<3,200>', '"heartbeat"','"cmd/led"','"chassis"']
    record('manual_equivalence',rc==0 and all(x in norm for x in expected) and
           norm.count('Motor4010<1>')==4 and norm.count('Motor4315<1>')==4 and
           all(norm.index('PIDwheel_'+p)<norm.index('Motor4010<1>C'+p+'Motor')<norm.index('autochassis=') for p in ['BR','BL','FL','FR']))
    t=copy.deepcopy(tree); block=t.find('./finemote_cpp'); block[:]=list(reversed(list(block)))
    r,s,_=generate('reverse_order',t); record('reverse_order',r==0 and s==source)
    t=copy.deepcopy(tree); p=pid(t); p[:]=list(reversed(list(p)))
    r,s,_=generate('field_order',t); record('field_order',r==0 and s==source)
    t=copy.deepcopy(tree); p=pid(t); p.remove(p.find("./field[@name='kd']"))
    r,s,_=generate('missing_kd',t); record('missing_kd',r==0 and 'PID_Param_t{0.23f, 0.008f}' in s)
    t=copy.deepcopy(tree); pid(t).find("./field[@name='kp']").set('name','pk')
    r,_,log=generate('unknown_field',t); record('unknown_field',r!=0 and 'pk' in log)
    t=copy.deepcopy(tree); p=pid(t); p.append(copy.deepcopy(p[0]))
    r,_,log=generate('duplicate_field',t); record('duplicate_field',r!=0 and 'kp' in log)
    t=copy.deepcopy(tree); ET.SubElement(obj(t,'CBRMotor'),'arg',name='unexpected',value='1')
    r,_,log=generate('extra_argument',t); record('extra_argument',r!=0 and 'unexpected' in log)
    t=copy.deepcopy(tree); pid(t).find("./field[@name='kp']").set('value','"bad"')
    r,_,log=generate('wrong_type',t); record('wrong_type',r!=0 and ('error:' in log or 'Error' in log))
    t=copy.deepcopy(tree); obj(t,'CBRMotor').find('./template').set('value','99')
    r,_,log=generate('invalid_bus',t); record('invalid_bus',r!=0 and 'Using illegal CAN BUS' in log)
    t=copy.deepcopy(tree); obj(t,'CBRMotor').find("./arg[@name='_controller']").set('ref','missing_device')
    r,_,log=generate('missing_reference',t); record('missing_reference',r!=0 and 'missing_device' in log)
    t=copy.deepcopy(tree); obj(t,'CBRMotor').find("./arg[@name='_controller']").set('ref','CBRMotor')
    r,_,log=generate('cycle',t); record('cycle',r!=0 and 'cycl' in log.lower())
    root=ET.Element('robot',name='generic'); block=ET.SubElement(root,'finemote_cpp')
    ET.SubElement(block,'include',header=str(ROOT/'Tests/URDFReflection/Probe.hpp').replace('\\','/'))
    probe=ET.SubElement(block,'object',id='probe',type='ReflectionProbe')
    ET.SubElement(probe,'template',name='N',value='7')
    arg=ET.SubElement(probe,'arg',name='config'); agg=ET.SubElement(arg,'aggregate')
    ET.SubElement(agg,'field',name='count',value='5'); ET.SubElement(agg,'field',name='gain',value='4.0f')
    r,s,_=generate('specialization_const',ET.ElementTree(root))
    record('specialization_const',r==0 and 'ReflectionConfig<7>{4.0f, 5}' in s and 'ReflectionProbe<7>' in s)
    record('constructor_default',r==0 and re.search(r'probe\{ReflectionConfig<7>\{4.0f, 5\}\}',s))
    t=copy.deepcopy(tree); pid(t).find("./field[@name='kp']").set('value','0.77f')
    changed=BUILD/'changed.urdf'; t.write(changed,encoding='utf-8',xml_declaration=True)
    r,_=command('change_input',['cmake','-S',ROOT,'-B',BUILD,'-DFINEMOTE_REFLECTED_URDF='+str(changed)])
    if r==0: r,_=command('rebuild',['cmake','--build',BUILD,'--parallel','6'])
    rebuilt=BUILD/'reflected/robot.cpp'
    record('urdf_rebuild',r==0 and rebuilt.exists() and '0.77f' in rebuilt.read_text(encoding='utf-8'))
    t=copy.deepcopy(tree); obj(t,'CBRMotor').set('type','NonexistentMotor')
    r,_,log=generate('unknown_model',t); record('unknown_model',r!=0 and 'NonexistentMotor' in log)
    t=copy.deepcopy(tree); ET.SubElement(obj(t,'CBRMotor'),'template',name='extra',value='1')
    r,_,log=generate('extra_template_argument',t); record('extra_template_argument',r!=0 and 'extra' in log)
    t=copy.deepcopy(tree); obj(t,'CBRMotor').find("./arg[@name='addr']").set('value','0x141')
    r,_,_=generate('shared_bus_id',t); record('shared_bus_id',r==0)
    r,symbols=command('symbols',[GCCBIN/'arm-none-eabi-nm.exe','-C',BUILD/'MC_Board.elf'])
    record('task_registration',r==0 and 'TaskPOVChassis' in symbols and 'make_task([] { TaskPOVChassis(); }, 1)' in source and
           'finemote_urdf::ZorroVelocityTask' in source and 'Decode' not in source and 'heartbeat' in source and 'led' in source)
try:
    main()
except Exception as e:
    import traceback
    traceback.print_exc()
finally:
    (BUILD/'results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
    print(sum(results.values()))
sys.exit(0 if all(results.values()) else 1)

"""Map an explicit URDF construction graph through Clang's real C++ AST.

No device/model/field registry is used. The accepted XML dialect is deliberately
small: named template/constructor arguments, aggregates, references and calls.
"""
from __future__ import annotations
import argparse, ctypes, hashlib, json, os, pathlib, re, shlex, subprocess, sys
import xml.etree.ElementTree as ET

class MappingError(Exception): pass
def require(ok, message):
    if not ok: raise MappingError(message)
def ident(s):
    require(bool(re.fullmatch(r'[A-Za-z][A-Za-z_0-9]*',s or '')),f'invalid identifier: {s!r}')
    return s
def typename(s):
    require(bool(re.fullmatch(r'[A-Za-z_][A-Za-z_0-9:<> ,*&]*',s or '')),f'invalid C++ type/function: {s!r}')
    return s
def literal(s):
    require(s is not None,'missing literal')
    if re.fullmatch(r'[+-]?(?:0[xX][0-9a-fA-F]+|(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)[uUlLfF]*',s): return s
    if s in ['true','false','nullptr']: return s
    if re.fullmatch(r'[A-Za-z_]\w*(?:::[A-Za-z_]\w*)+',s): return s
    try:
        require(isinstance(json.loads(s),str),'expected a quoted string')
        require('\n' not in s and '\r' not in s,'multiline literal')
        return s
    except (ValueError,MappingError): raise MappingError(f'unsupported literal {s!r}; use a scalar, enum or reference')
def children(n): return n.get('inner',[])
def params(n): return [c for c in children(n) if c['kind']=='ParmVarDecl']
def type_of(n): return n.get('type',{}).get('desugaredQualType',n.get('type',{}).get('qualType',''))
def clean_type(s): return re.sub(r'\b(const|volatile|struct|class)\s*','',s).rstrip(' &')
def split_command(s):
    if os.name!='nt': return shlex.split(s)
    shell=ctypes.windll.shell32
    shell.CommandLineToArgvW.argtypes=[ctypes.c_wchar_p,ctypes.POINTER(ctypes.c_int)]
    shell.CommandLineToArgvW.restype=ctypes.POINTER(ctypes.c_wchar_p)
    count=ctypes.c_int(); argv=shell.CommandLineToArgvW(s,ctypes.byref(count))
    try: return [argv[i] for i in range(count.value)]
    finally: ctypes.windll.kernel32.LocalFree(argv)
def decode_stream(text):
    decoder=json.JSONDecoder(); result=[]; pos=0
    while pos<len(text):
        while pos<len(text) and text[pos].isspace(): pos+=1
        if pos==len(text): break
        obj,pos=decoder.raw_decode(text,pos); result.append(obj)
    return result

class Reflection:
    def __init__(self,db,clang,headers,out):
        entries=json.loads(db.read_text(encoding='utf-8'))
        entry=next((e for e in entries if e['file'].replace('\\','/').endswith('Interface/Interface.cpp')),None)
        require(entry is not None,'compile database lacks Interface.cpp context')
        cmd=entry.get('arguments') or split_command(entry['command'])
        self.compiler=cmd[0]; self.cwd=entry['directory']; self.clang=clang
        self.flags=[]; skip=False
        for token in cmd[1:]:
            if skip: skip=False; continue
            if token in ['-o','-MF','-MT','-MQ']: skip=True; continue
            if token in ['-c','-MD','-MMD'] or token.startswith('-std='): continue
            if token==entry['file'] or token.replace('\\','/')==entry['file'].replace('\\','/'): continue
            self.flags.append(token)
        self.flags.append('-std=c++17')
        self.headers=headers
        self.prelude='#include <cstddef>\n#include <vector>\n#include <type_traits>\n#include <utility>\n'
        self.prelude+=''.join('#include "'+h+'"\n' for h in headers)
        self.folder=out.parent/'ast'; self.folder.mkdir(parents=True,exist_ok=True)
        self.cache={}; self.used={}
        self.root=pathlib.Path(__file__).resolve().parents[2]
        # Cache invalidation includes headers, compiler identity, flags and include set.
        digest=hashlib.sha256((self.prelude+json.dumps(self.flags)+str(pathlib.Path(clang).stat())).encode())
        for folder in ['Algorithms','Components','Devices','Services','Interface','BSP','Tests']:
            for p in sorted((self.root/folder).rglob('*')):
                if p.is_file() and p.suffix in ['.h','.hpp','.h++']:
                    digest.update(str(p).encode()); digest.update(p.read_bytes())
        for h in headers:
            p=pathlib.Path(h)
            if p.is_absolute() and p.is_file(): digest.update(p.read_bytes())
        self.key=digest.hexdigest()
        self.disk=db.parent/'ast-interface-cache'; self.disk.mkdir(exist_ok=True)
    def dump(self,name,concrete=None):
        key=(name,concrete)
        if key in self.cache: return self.cache[key]
        filename=hashlib.sha256((self.key+repr(key)).encode()).hexdigest()
        cached=self.disk/(filename+'.json')
        if cached.exists():
            nodes=json.loads(cached.read_text(encoding='utf-8'))
        else:
            source=self.folder/(filename+'.cpp')
            extra=('using __fm_reflected_type = '+concrete+';\nstatic_assert(sizeof(__fm_reflected_type)>0);\n') if concrete else ''
            source.write_text(self.prelude+extra,encoding='utf-8')
            flags=[f for f in self.flags if f not in ['-fno-rtti','-fno-exceptions']]
            target='--target=arm-arm-none-eabi' if 'armclang' in pathlib.Path(self.clang).name else '--target=arm-none-eabi'
            command=[self.clang,target,*flags,'-fsyntax-only','-Xclang','-ast-dump=json',
                     '-Xclang','-ast-dump-filter='+name,str(source)]
            p=subprocess.run(command,cwd=self.cwd,capture_output=True)
            (self.folder/(filename+'.diagnostics.txt')).write_bytes(p.stderr)
            require(p.returncode==0,'Clang reflection failed for '+str(concrete or name)+':\n'+p.stderr.decode('utf-8',errors='replace'))
            nodes=decode_stream(p.stdout.decode('utf-8'))
            cached.write_text(json.dumps(nodes),encoding='utf-8')
        self.cache[key]=nodes
        return nodes
    def describe(self,name,concrete=None,function=False):
        nodes=self.dump(name,concrete)
        short=name.split('::')[-1]
        candidates=[n for n in nodes if n.get('name')==short and not n.get('isImplicit')]
        require(candidates,f'no declaration found for {name}')
        templates=[]; declarations=[]
        for n in candidates:
            if n['kind'] in ['ClassTemplateDecl','FunctionTemplateDecl']:
                templates=[c for c in children(n) if c['kind'] in ['TemplateTypeParmDecl','NonTypeTemplateParmDecl','TemplateTemplateParmDecl']]
                declarations.extend(c for c in children(n) if c['kind'] in ['CXXRecordDecl','ClassTemplateSpecializationDecl','FunctionDecl'])
            else: declarations.append(n)
        if function:
            calls=[n for n in declarations if n['kind']=='FunctionDecl' and not n.get('isImplicit')]
            result={'templates':templates,'calls':calls,'fields':[],'record':None}
        else:
            records=[n for n in declarations if n.get('completeDefinition') and n['kind'] in ['CXXRecordDecl','ClassTemplateSpecializationDecl']]
            if concrete and '<' in concrete:
                specs=[n for n in records if n['kind']=='ClassTemplateSpecializationDecl']
                if specs:
                    wanted=concrete[concrete.index('<')+1:-1].replace(' ','')
                    def arguments(n):
                        return ','.join(str(c.get('value',type_of(c))) for c in children(n) if c['kind']=='TemplateArgument').replace(' ','')
                    matches=[n for n in specs if arguments(n)==wanted]
                    records=matches or specs
            require(records,f'no complete class declaration found for {concrete or name}')
            record=records[-1] if concrete else records[0]
            calls=[]; fields=[]; access='public' if record.get('tagUsed')=='struct' else 'private'
            for c in children(record):
                if c['kind']=='AccessSpecDecl': access=c['access']
                if access!='public': continue
                if c['kind']=='FieldDecl': fields.append(c)
                if c['kind']=='CXXConstructorDecl' and not c.get('isImplicit'): calls.append(c)
                if c['kind']=='FunctionTemplateDecl':
                    calls.extend(x for x in children(c) if x['kind']=='CXXConstructorDecl' and not x.get('isImplicit'))
            # Clang defers instantiating default arguments. Recover their presence
            # from the matching source declaration, while keeping concrete types.
            defaults=set()
            def collect_defaults(node):
                if node.get('kind')=='ParmVarDecl' and node.get('init'):
                    defaults.add((node.get('name'),node.get('loc',{}).get('offset')))
                for child in children(node): collect_defaults(child)
            for primary in declarations:
                if primary['kind']=='CXXRecordDecl': collect_defaults(primary)
            for call in calls:
                for parameter in params(call):
                    if (parameter.get('name'),parameter.get('loc',{}).get('offset')) in defaults:
                        parameter.setdefault('init','inherited-from-template-declaration')
            result={'templates':templates,'calls':calls,'fields':fields,'record':record}
        self.used[str(concrete or name)]={
            'templates':[{'name':n.get('name'),'kind':n['kind'],'pack':bool(n.get('isParameterPack'))} for n in templates],
            'fields':[{'name':n['name'],'type':type_of(n),'default':bool(n.get('hasInClassInitializer'))} for n in result['fields']],
            'constructors_or_functions':[[{'name':p.get('name'),'type':type_of(p),'default':bool(p.get('init')),'pack':bool(p.get('isParameterPack'))} for p in params(n)] for n in result['calls']],
            'declaration':(result['record'] or candidates[0]).get('loc',{})}
        return result
    def check(self,path):
        p=subprocess.run([self.compiler,*self.flags,'-fsyntax-only',str(path)],cwd=self.cwd,capture_output=True)
        require(p.returncode==0,'Target C++17 compiler rejected generated construction:\n'+p.stderr.decode('utf-8',errors='replace'))

class Generator:
    def __init__(self,tree,reflection):
        self.tree=tree; self.reflection=reflection
        block=tree.find('./finemote_cpp'); require(block is not None,'missing finemote_cpp extension')
        require(set(block.attrib)<= {'version'},'unknown finemote_cpp attribute')
        vocabulary={
            'finemote_cpp':({'version'},{'include','object','value','task'}),
            'include':({'header'},set()),
            'object':({'id','type','factory'},{'template','arg'}),
            'value':({'id','type','value'},set()),
            'task':({'name','object','method'},set()),
            'template':({'name','value','type'},set()),
            'arg':({'name','value','ref','address','joint','coordinate'},{'aggregate','call'}),
            'field':({'name','value','ref','address','joint','coordinate'},{'aggregate','call'}),
            'aggregate':({'type'},{'field'}),
            'call':({'function'},{'template','arg'})}
        for node in block.iter():
            require(node.tag in vocabulary,'unknown extension element '+node.tag)
            attrs,tags=vocabulary[node.tag]
            require(set(node.attrib)<=attrs,'unknown '+node.tag+' attributes: '+str(set(node.attrib)-attrs))
            require(all(c.tag in tags for c in node),'unsupported child in '+node.tag)
            require(not (node.text or '').strip(),'unexpected text in '+node.tag)
            if node.tag=='object': require(('type' in node.attrib)!=('factory' in node.attrib),'object needs exactly one type or factory')
        self.objects={}; self.tasks=[]
        for n in block:
            require(n.tag in ['include','object','value','task'],'unknown extension element '+n.tag)
            if n.tag in ['object','value']:
                name=ident(n.get('id')); require(name not in self.objects,'duplicate object '+name); self.objects[name]=n
            if n.tag=='task': self.tasks.append(n)
        self.deps={name:{e.get('ref') for e in node.iter() if e.get('ref') is not None} for name,node in self.objects.items()}
        for name,refs in self.deps.items():
            require(refs<=self.objects.keys(),f'{name}: missing reference {sorted(refs-self.objects.keys())}')
        self.order=[]; pending=set(self.objects)
        while pending:
            ready=sorted(n for n in pending if self.deps[n]<=set(self.order))
            require(ready,'dependency cycle among '+', '.join(sorted(pending)))
            self.order.extend(ready); pending.difference_update(ready)
    def template_type(self,n,name,function=False):
        info=self.reflection.describe(name,function=function)
        supplied=list(n.findall('./template'))
        if not supplied: return name,info
        names=[x.get('name') for x in info['templates']]
        require(len({x.get('name') for x in supplied})==len(supplied),'duplicate template argument')
        values={x.get('name'):x for x in supplied}
        require(values.keys()<=set(names),'unknown template argument '+', '.join(values.keys()-set(names)))
        args=[]; omitted=False
        for p in info['templates']:
            key=p.get('name')
            if key not in values:
                omitted=True; continue
            require(not omitted,'interior omitted template argument before '+str(key))
            item=values[key]
            require(set(item.attrib)<= {'name','value','type'},'unknown template attributes')
            args.append(typename(item.get('type')) if item.get('type') else literal(item.get('value')))
        concrete=name+'<'+', '.join(args)+'>'
        if not function: info=self.reflection.describe(name,concrete)
        return concrete,info
    def expression(self,n,expected=None):
        modes=[k for k in ['value','ref','joint'] if k in n.attrib]
        require(len(modes)+len(n)==1,f'{n.tag} needs exactly one value, reference, joint coordinate or nested expression')
        if 'value' in n.attrib: return literal(n.get('value'))
        if 'ref' in n.attrib:
            require(n.get('address','false') in ['true','false'],'address must be true or false')
            return ('&' if n.get('address')=='true' else '')+ident(n.get('ref'))
        if 'joint' in n.attrib:
            matches=[j for j in self.tree.findall('./joint') if j.get('name')==n.get('joint')]
            require(len(matches)==1,'unknown/ambiguous joint '+n.get('joint'))
            coord=n.get('coordinate'); require(coord in ['x','y','z'],'coordinate must be x/y/z')
            origin=matches[0].find('./origin'); xyz=(origin.get('xyz','0 0 0') if origin is not None else '0 0 0').split()
            require(len(xyz)==3,'joint origin xyz must have three values')
            value=xyz[['x','y','z'].index(coord)]
            literal(value)
            return value+('f' if '.' in value or 'e' in value.lower() else '.0f')
        child=n[0]
        if child.tag=='aggregate': return self.aggregate(child,expected)
        if child.tag=='call': return self.construct(child,True)
        raise MappingError('unknown expression '+child.tag)
    def aggregate(self,n,expected):
        typ=typename(n.get('type') or clean_type(expected or ''))
        info=self.reflection.describe(typ.split('<')[0],typ if '<' in typ else None)
        require(info['record'].get('definitionData',{}).get('isAggregate'),typ+' is not an aggregate')
        fields=info['fields']; supplied={}
        for f in n:
            require(f.tag=='field','expected aggregate field')
            name=f.get('name'); require(name not in supplied,'duplicate field '+str(name)); supplied[name]=f
        names=[f['name'] for f in fields]
        require(supplied.keys()<=set(names),typ+': unknown field '+', '.join(supplied.keys()-set(names)))
        last=max((names.index(name) for name in supplied),default=-1)
        values=[]
        for f in fields[:last+1]:
            if f['name'] in supplied: values.append(self.expression(supplied[f['name']],type_of(f)))
            else:
                require(not f.get('hasInClassInitializer'),f"{typ}.{f['name']}: interior default initializer needs explicit value in this prototype")
                values.append('{}')
        return typ+'{'+', '.join(values)+'}'
    def construct(self,n,function=False):
        name=typename(n.get('function') if function else n.get('factory') or n.get('type'))
        function=function or n.get('factory') is not None
        typ,info=self.template_type(n,name,function)
        args=n.findall('./arg'); supplied={}
        for a in args: supplied.setdefault(a.get('name'),[]).append(a)
        candidates=[]
        for c in info['calls']:
            ps=params(c); names={p.get('name') for p in ps}
            if not supplied.keys()<=names: continue
            if any(p.get('name') not in supplied and not p.get('init') and not p.get('isParameterPack') for p in ps): continue
            candidates.append(c)
        # Clang can report a declaration and its instantiated definition twice.
        unique={tuple((p.get('name'),type_of(p)) for p in params(c)):c for c in candidates}
        candidates=list(unique.values())
        require(len(candidates)==1,f'{typ}: cannot uniquely map arguments {list(supplied)} to public '+('function' if function else 'constructor'))
        values=[]; omitted=False
        for p in params(candidates[0]):
            seq=supplied.get(p.get('name'),[])
            if not seq:
                omitted=True; continue
            require(not omitted,'interior omitted constructor argument before '+str(p.get('name')))
            require(len(seq)==1 or p.get('isParameterPack'),'duplicate argument '+str(p.get('name')))
            values.extend(self.expression(a,type_of(p)) for a in seq)
        return typ+('('+', '.join(values)+')' if function else '{'+', '.join(values)+'}')
    def generate(self):
        lines=['// Generated from URDF and real Clang declarations. Do not edit.',self.reflection.prelude,
               '#include "DeviceBase/Task.hpp"','static_assert(__cplusplus == 201703L, "Generated firmware requires C++17");','namespace {']
        for name in self.order:
            n=self.objects[name]
            if n.tag=='value':
                lines.append(typename(n.get('type'))+' '+name+' = '+literal(n.get('value'))+';')
            else:
                expression=self.construct(n)
                if n.get('factory'): lines.append('auto '+name+' = '+expression+';')
                else:
                    pos=expression.index('{'); lines.append(expression[:pos]+' '+name+expression[pos:]+';')
        lines.append('} // namespace')
        task_names=set()
        for t in sorted(self.tasks,key=lambda t:t.get('name','')):
            name=ident(t.get('name')); obj=ident(t.get('object')); method=ident(t.get('method'))
            require(name not in task_names and name not in self.objects,'duplicate task '+name); task_names.add(name)
            require(obj in self.objects,'missing task object '+obj)
            lines.extend(['void '+name+'() { '+obj+'.'+method+'(); }',
                          '[[maybe_unused]] static auto& registered_'+name+' = make_task([] { '+name+'(); }, 1);'])
        return '\n'.join(lines)+'\n'

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['urdf','compile-db','output','metadata']: parser.add_argument('--'+name,type=pathlib.Path,required=True)
    parser.add_argument('--clang',required=True); parser.add_argument('--check',action='store_true')
    parser.add_argument('--controllers',type=pathlib.Path,help='controller YAML for the FineMote device description')
    args=parser.parse_args()
    for attr in ['urdf','compile_db','output','metadata']:
        setattr(args,attr,getattr(args,attr).resolve())
    tree=ET.parse(args.urdf); block=tree.find('./finemote_cpp')
    manifest={}
    if args.controllers:
        from devices import device_graph, load_controllers, ros_source
        require(block is None,'cannot combine finemote_cpp with controller YAML')
        block,manifest=device_graph(tree.getroot(),load_controllers(args.controllers))
        communication=ros_source(manifest)
        tree.getroot().append(block)
    require(block is not None,'missing finemote_cpp extension (device descriptions require --controllers)')
    headers=sorted({n.get('header','') for n in block.findall('./include')})
    require(all(h and '"' not in h and '\n' not in h and '\r' not in h for h in headers),'invalid include header')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    reflection=Reflection(args.compile_db,args.clang,headers,args.output)
    generator=Generator(tree,reflection); code=generator.generate()
    if args.controllers: code+=communication
    args.output.write_text(code,encoding='utf-8')
    args.metadata.write_text(json.dumps({'standard':'c++17','interfaces':reflection.used,'construction_order':generator.order,'dependencies':{k:sorted(v) for k,v in generator.deps.items()},**manifest},ensure_ascii=False,indent=2),encoding='utf-8')
    if args.check: reflection.check(args.output)
    print('Generated '+str(args.output))
if __name__=='__main__':
    try: main()
    except (MappingError,ET.ParseError,OSError,ValueError) as e:
        print('URDF mapping error: '+str(e),file=sys.stderr); sys.exit(1)

import inspect
import numpy as np
import math
import random
from base import *
import util
import os
import pathlib
import random
import importlib
from collections import namedtuple

def create_core(module):
    c = module.BirdsCore()
    c.init_simulation()
    return c, c.reference_sim_parameters()

def _prepare(module):
    core, simp = create_core(module)
    return core, simp

def _ref_prepare():
    try:
        from pypsim_ref.bird1 import BirdsCore as RefBirdsCore
    except ModuleNotFoundError as e:
        print(e)
        print("This script should be executed under bin/. Example")
        print("psim/bin$ ../grading/psim_grade.py --case bird1 .....")
        exit()
    c = RefBirdsCore()
    c.init_simulation()
    return c, c.reference_sim_parameters()

def _prepare_bicore(module):
    core,_ = _prepare(module)
    refcore,_ = _ref_prepare()
    bicore = BiCore(module, refcore)

    bicore.set_simp('dampingEnabled', False)
    return bicore

'''
Rigid Body Template
'''
class BiRBT(object):
    def __init__(self, module, fn):
        from pypsim_ref.bird1 import RigidBodyTemplate as RefRBT
        from pypsim_ref.bird1 import loadOBJ
        V, F = loadOBJ(fn)
        SCALE = 3.5
        self.rbt = module.RigidBodyTemplate(V, F, SCALE)
        self.ref_rbt = RefRBT(V, F, SCALE)
        self.verts = self.ref_rbt.getVerts()
        self.tets = self.ref_rbt.getTets()

    def compare(self, name, atol=1e-3, rtol=1e-3):
        v = getattr(self.rbt, name)
        refv = getattr(self.ref_rbt, name)
        compare_result = np.allclose(v, refv, atol=atol, rtol=rtol)
        print(f'comparing {name}, result: {compare_result}. value: {v}, expect {refv}')
        return compare_result

def get_rbt_models():
    print(__file__)
    script_path = pathlib.Path(os.path.abspath(__file__))
    # print(f"script_path {script_path}")
    model_dir = script_path.parent.joinpath('../assets/birds/')
    # print(f"model_dir {model_dir}")
    models = list(model_dir.glob("*.obj"))
    assert len(models) > 0
    # print(models)
    return models

'''
4.1 Rigid Body Template
'''
class Volume(TestBase):
    PTS = 5
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        for p in get_rbt_models():
            fn = p.stem
            ret.add_result(BiRBT(module, str(p)).compare('volume'), name=f"{fn} volume")

class CenterOfMass(TestBase):
    PTS = 5
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        for p in get_rbt_models():
            fn = p.stem
            ret.add_result(BiRBT(module, str(p)).compare('com'), name=f"{fn} Center of Mass")

class Centralized(TestBase):
    PTS = 5
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        SCALE = 2.3
        from pypsim_ref.bird1 import RigidBodyTemplate as RefRBT, loadOBJ
        for p in get_rbt_models():
            fn = p.stem
            V, F = loadOBJ(str(p))
            rbt = module.RigidBodyTemplate(V, F, SCALE)
            nV = rbt.getVerts()
            nF = rbt.getFaces()
            ref_rbt = RefRBT(nV, nF, SCALE)
            passed = np.allclose(ref_rbt.com, np.array([0,0,0]))
            ret.add_result(passed, name=f"{fn} Translate CoM to Origin")

class InertiaTensor(TestBase):
    PTS = 10
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        for p in get_rbt_models():
            fn = p.stem
            ret.add_result(BiRBT(module, str(p)).compare('MI'), name=f"{fn} Inertia Tensor")

'''
End Of Testing Rigid Body Templates
'''

RigidBodyParameters = namedtuple('RigidBodyParameters',
                                 ['density',
                                  'center', 'rotation',
                                  'center_velocity', 'angular_velocity'])

class BiCore(object):
    def __init__(self, module, ref_module):
        self.module = module
        self.ref_module = ref_module
        self.core, self.simp = _prepare(module)
        self.refcore, self.refsimp = _prepare(ref_module)
        self.inst_ids = []
        self.ref_inst_ids = []
        self.rbt_storage = [] # Must otherwise RBT may be recycled

    def set_simp(self, name, val):
        setattr(self.simp, name, val)
        setattr(self.refsimp, name, val)

    def add_rbt(self, fn, scale):
        V, F = self.ref_module.loadOBJ(fn)
        rbt = self.module.RigidBodyTemplate(V, F, scale)
        ref_rbt = self.ref_module.RigidBodyTemplate(V, F, scale)
        pair = (rbt, ref_rbt)
        self.rbt_storage.append(pair)
        return pair

    def add_instance(self, rbt_pair, param):
        rbt, ref_rbt = rbt_pair
        inst_id = self.core.add_single_instance(rbt,
                                                param.density,
                                                param.center,
                                                param.rotation,
                                                param.center_velocity,
                                                param.angular_velocity)
        ref_inst_id = self.refcore.add_single_instance(ref_rbt,
                                                       param.density,
                                                       param.center,
                                                       param.rotation,
                                                       param.center_velocity,
                                                       param.angular_velocity)
        self.inst_ids.append(inst_id)
        self.ref_inst_ids.append(ref_inst_id)

    def simulate_one_step(self):
        self.core.simulate_one_step()
        self.refcore.simulate_one_step()

    def compare(self, pid_pair, attr_name, rtol=1e-3):
        pid_vec, refpid_vec = pid_pair
        for pid, refpid in zip(pid_vec, refpid_vec):
            p, valid = self.core.query_particle(pid)
            refp, refvalid = self.refcore.query_particle(refpid)
            if valid != refvalid:
                print(f"valid ({valid}) != refvalid ({refvalid})")
                return False
            if not np.allclose(p.mass, refp.mass):
                print(f"mass ({p.mass}) != refmass ({refp.mass})")
                return False
            if not np.allclose(p.pos, refp.pos, rtol=rtol):
                print(f"pos ({p.pos}) != refpos ({refp.pos})")
                return False
            if not np.allclose(p.vel, refp.vel, rtol=rtol):
                print(f"vel ({p.vel}) != refvel ({refp.vel})")
                return False
        return True

    def compare_result_gen(self, attr_name, atol=1e-3, rtol=1e-3):
        for inst_id, ref_inst_id in zip(self.inst_ids, self.ref_inst_ids):
            rbi = self.core.query_rigid_body_instance(inst_id)
            ref_rbi = self.refcore.query_rigid_body_instance(ref_inst_id)
            v = getattr(rbi, attr_name)
            refv = getattr(ref_rbi, attr_name)
            compare_result = np.allclose(v, refv, atol=atol, rtol=rtol)
            print(f'comparing {attr_name}, result: {compare_result}. value: {v}, expect {refv}')
            yield inst_id, ref_inst_id, compare_result
        print('comparison for current itertion done')

    def sync_with_reference(self):
        for inst_id, ref_inst_id in zip(self.inst_ids, self.ref_inst_ids):
            rbi = self.core.query_rigid_body_instance(inst_id)
            ref_rbi = self.refcore.query_rigid_body_instance(ref_inst_id)
            rbi.c = ref_rbi.c
            rbi.theta = ref_rbi.theta
            rbi.cvel = ref_rbi.cvel
            rbi.w = ref_rbi.w

    def reset(self):
        self.core.init_simulation()
        self.refcore.init_simulation()

def _randunitvec3():
    phi = random.uniform(0, math.pi*2)
    costheta = random.uniform(-1,1)

    theta = np.arccos(costheta)
    x = np.sin(theta) * np.cos(phi)
    y = np.sin(theta) * np.sin(phi)
    z = np.cos(theta)
    return np.array([x,y,z])

def _randvec3(lo=-1000.0, hi=1000.0, randmag=None):
    if randmag is None:
        return np.array([random.uniform(lo, hi) for i in range(3)])
    ln_mag = random.uniform(np.log(0.01), np.log(randmag))
    mag = np.exp(ln_mag)
    return _randunitvec3() * mag


def bicore_ti_gen(module, ref_module, single_instance=True, disable_gravity=True):
    '''
    Seed python's random to make it determinstic
    The string is dumped from /dev/urandom at the developer's system
    '''
    random.seed(bytes.fromhex('36f8a1c44c7fb402caf756d6eafab4ea'))
    rbt_fns = get_rbt_models()

    bicore = None

    for p in rbt_fns:
        if single_instance or bicore is None:
            bicore = BiCore(module, ref_module)
            bicore.set_simp("NewtonMaxIters", 20)
            bicore.set_simp("NewtonTolerance", 1e-9)
            if disable_gravity:
                bicore.set_simp("gravityEnabled", False)
                bicore.set_simp("gravityG", 0.0)
            else:
                bicore.set_simp("gravityEnabled", True)
                bicore.set_simp("gravityG", 0.98)

        fn = str(p)
        ln_scale = random.uniform(np.log(0.1), np.log(1))
        scale = np.exp(ln_scale)
        rbt_pair = bicore.add_rbt(fn, scale=scale)
        param = RigidBodyParameters(density=random.uniform(1, 100),
                                    center=_randvec3(),
                                    rotation=_randvec3(randmag=math.pi),
                                    center_velocity=_randvec3(),
                                    angular_velocity=_randvec3(randmag=math.pi))
        bicore.add_instance(rbt_pair, param)
        if single_instance:
            yield bicore
    if not single_instance:
        yield bicore

'''
4.2 Time Integration

Create a pendulum and verify first 100 steps
'''

def test_integrator_common(module, ret, attr_name, single_instance=True, disable_gravity=True,
                           atol=1e-3, rtol=1e-3):
    ref_module = importlib.import_module('pypsim_ref.bird1')
    for bicore in bicore_ti_gen(module, ref_module,
                                single_instance=single_instance,
                                disable_gravity=disable_gravity):
        for i in range(100):
            bicore.simulate_one_step()
            for _,_,ok in bicore.compare_result_gen(attr_name=attr_name, atol=atol, rtol=rtol):
                ret.add_result(ok)
            bicore.sync_with_reference()

class TimeIntegratorCenterOfMass(TestBase):
    PTS = 12.5
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        test_integrator_common(module, ret, attr_name='c')

class TimeIntegratorRotation(TestBase):
    PTS = 12.5
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        test_integrator_common(module, ret, attr_name='theta')

class TimeIntegratorVelocity(TestBase):
    PTS = 12.5
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        test_integrator_common(module, ret, attr_name='cvel')

class TimeIntegratorAngularVelocity(TestBase):
    PTS = 12.5
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        test_integrator_common(module, ret, attr_name='w')

'''
4.3 Gravity

Create a pendulum and verify first 100 steps
'''

class Gravity(TestBase):
    PTS = 25
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        test_integrator_common(module, ret, attr_name='c', single_instance=False,
                               disable_gravity=False, atol=1e-3, rtol=1e-3)
        test_integrator_common(module, ret, attr_name='cvel', single_instance=False,
                               disable_gravity=False, atol=1e-3, rtol=1e-2)

'''
Extra credit shall be added manually
'''
class Extra(TestBase):
    PTS = 15
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        ret.add_result(False)

'''
Auxiliary functions
'''
def get_test_objects():
    test_objects = []
    g = globals().copy()
    for name, obj in g.items():
        # print(f"Find test class {name}")
        if not inspect.isclass(obj):
            continue
        if not issubclass(obj, TestBase) or obj == TestBase:
            continue
        print(f"Find test class {name}")
        test_objects.append(obj())
    total_pts = np.sum([t.report_total_points() for t in test_objects])
    assert total_pts == 100 or total_pts == 115, f'Total points {total_pts} != 100 or 115'
    return test_objects

def test_object_generator(args):
    test_objects = get_test_objects()
    for obj in test_objects:
        klass = type(obj)
        kname = klass.__name__
        if hasattr(args, 'k') and args.k and args.k != kname:
            continue
        yield obj, kname

def grade(args, module):
    pts_dic = {}
    ttl_dic = {}
    for t, kname in test_object_generator(args):
        print(f'Grading {kname}', end='')
        result = t.test(module)
        pts = result.points()
        ttl = t.report_total_points()
        print(f'\tPoints {pts}/{ttl}')
        if args.verbose and not np.allclose(pts, ttl):
            print(result)
        pts_dic[kname] = pts
        ttl_dic[kname] = ttl
    return pts_dic, ttl_dic

def rubrics(args):
    test_objects = get_test_objects()
    rub_list = []
    for t, kname in test_object_generator(args):
        tname = util.insert_spaces_to_camel(kname) if not hasattr(t, 'RUBIRIC_NAME') else getattr(t, 'RUBIRIC_NAME')
        rub_list.append((tname, t.report_total_points(), kname))
    return rub_list

import inspect
import numpy as np
import math
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
        from pypsim_ref.bird2 import BirdsCore as RefBirdsCore
    except ModuleNotFoundError as e:
        print(e)
        print("This script should be executed under bin/. Example")
        print("psim/bin$ ../grading/psim_grade.py --case bird2 .....")
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
        from pypsim_ref.bird2 import RigidBodyTemplate as RefRBT
        from pypsim_ref.bird2 import loadOBJ
        d = np.load(fn)
        SCALE = 3.5
        V = d['V'] * SCALE
        T = d['T']
        self.rbt = module.RigidBodyTemplate(V, T)
        self.ref_rbt = RefRBT(V, T)
        self.verts = self.ref_rbt.getVerts()
        self.tets = self.ref_rbt.getTets()

    def compare(self, name, atol=1e-3, rtol=1e-3):
        v = getattr(self.rbt, name)
        refv = getattr(self.ref_rbt, name)
        compare_result = np.allclose(v, refv, atol=atol, rtol=rtol)
        print(f'comparing {name}, result: {compare_result}. value: {v}, expect {refv}')
        return compare_result

    def compare_distance(self, coord, tet_index, atol=1e-3, rtol=1e-3):
        '''
        Apply abs() to the return value. The sign is not important
        '''
        v = abs(self.rbt.distance(coord, tet_index))
        refv = abs(self.ref_rbt.distance(coord, tet_index))
        compare_result = np.allclose(v, refv, atol=atol, rtol=rtol)
        # print(f'comparing distance, result: {compare_result}. value: {v}, expect {refv}')
        return compare_result

    def compare_jacobian(self, tet_index, atol=1e-3, rtol=1e-3):
        v = self.rbt.getJacobian(tet_index)
        refv = self.ref_rbt.getJacobian(tet_index)
        compare_result = np.allclose(v, refv, atol=atol, rtol=rtol) or np.allclose(-v, refv, atol=atol, rtol=rtol)
        # print(f'comparing distance, result: {compare_result}. value: {v}, expect {refv}')
        return compare_result

def get_rbt_models():
    print(__file__)
    script_path = pathlib.Path(os.path.abspath(__file__))
    # print(f"script_path {script_path}")
    model_dir = script_path.parent.joinpath('../assets/birds/')
    # print(f"model_dir {model_dir}")
    models = list(model_dir.glob("*.npz"))
    assert len(models) > 0
    # print(models)
    return models

RigidBodyParameters = namedtuple('RigidBodyParameters',
                                 ['density',
                                  'center', 'rotation',
                                  'center_velocity', 'angular_velocity'])

def _add_rbi(core, rbt, param):
    return core.add_single_instance(rbt,
                                    param.density,
                                    param.center,
                                    param.rotation,
                                    param.center_velocity,
                                    param.angular_velocity)

def _sync_rbi(target_rbi, source_rbi):
    target_rbi.c = source_rbi.c
    target_rbi.theta = source_rbi.theta
    target_rbi.cvel = source_rbi.cvel
    target_rbi.w = source_rbi.w

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

    def add_rbt_from_npz(self, fn, scale):
        d = np.load(fn)
        V = d['V'] * scale
        T = d['T']
        rbt = self.module.RigidBodyTemplate(V, T)
        ref_rbt = self.ref_module.RigidBodyTemplate(V, T)
        pair = (rbt, ref_rbt)
        self.rbt_storage.append(pair)
        return pair

    def add_instance(self, rbt_pair, param):
        rbt, ref_rbt = rbt_pair
        inst_id = _add_rbi(self.core, rbt, param)
        ref_inst_id = _add_rbi(self.refcore, ref_rbt, param)
        self.inst_ids.append(inst_id)
        self.ref_inst_ids.append(ref_inst_id)

    def get_instance_pair_at(self, index):
        assert 0 <= index < len(self.inst_ids)
        rbi = self.core.query_rigid_body_instance(self.inst_ids[index])
        ref_rbi = self.refcore.query_rigid_body_instance(self.ref_inst_ids[index])
        return rbi, ref_rbi

    def simulate_one_step(self):
        self.core.simulate_one_step()
        self.refcore.simulate_one_step()

    def backup_system_state(self):
        rbi_states = [_read_rbi(rbi) for rbi in self.core.list_rigid_body_instances()]
        ref_rbi_states = [_read_rbi(rbi) for rbi in self.refcore.list_rigid_body_instances()]
        return rbi_states, ref_rbi_states

    def restore_system_state(self, backup):
        rbi_states, ref_rbi_states = backup
        for rbi, state in zip(self.core.list_rigid_body_instances(), rbi_states):
            _update_rbi(rbi, state)
        for rbi, state in zip(self.refcore.list_rigid_body_instances(), ref_rbi_states):
            _update_rbi(rbi, state)

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

    def compare_penalty_force(self,
                              compare_cforce, compare_theta_force,
                              atol=1e-3, rtol=1e-3):
        SHAPE = (3*len(self.inst_ids))
        cforce = np.zeros(SHAPE)
        thetaforce = np.zeros(SHAPE)
        ref_cforce = np.zeros(SHAPE)
        ref_thetaforce = np.zeros(SHAPE)

        ref_rbi_list = self.refcore.list_rigid_body_instances()
        # print(f'ref_rbi_list {len(ref_rbi_list)}')
        # print(XXX)
        col = self.module.detect_collision(self.core.list_rigid_body_instances())
        ref_col = self.ref_module.detect_collision(ref_rbi_list)

        # print(f'ref_col {ref_col}')

        all_compare_results = []
        for col_e, ref_col_e in zip(col, ref_col):
            col_e.body1 = ref_col_e.body1
            col_e.body2 = ref_col_e.body2
            col_e.collidingVertex = ref_col_e.collidingVertex
            col_e.collidingTet = ref_col_e.collidingTet
            col_pick = set([col_e])
            ref_col_pick = set([ref_col_e])
            # print(f'col {col}')
            self.core.compute_penalty_collision_forces(col_pick, cforce, thetaforce)
            self.refcore.compute_penalty_collision_forces(ref_col_pick, ref_cforce, ref_thetaforce)
            compare_result = True
            if compare_cforce:
                # print(f'[compare] col {len(col)}')
                # print(f'[compare] ref_col {len(ref_col)}')
                print(f'[compare] cforce {cforce} ref_cforce {ref_cforce}')
                compare_result = compare_result and (np.allclose(cforce, ref_cforce, atol=atol, rtol=rtol) or np.allclose(cforce, -ref_cforce, atol=atol, rtol=rtol))
            if compare_theta_force:
                # print(f'[compare] thetaforce {thetaforce} ref_thetaforce {ref_thetaforce}')
                compare_result = compare_result and (np.allclose(thetaforce, ref_thetaforce, atol=atol, rtol=rtol) or np.allclose(thetaforce, -ref_thetaforce, atol=atol, rtol=rtol))
            all_compare_results.append(compare_result)
        # print(f'all_compare_results {all_compare_results}')
        return all_compare_results

    def compare_impulse(self, atol=1e-2, rtol=1e-2):
        rb_state = self.backup_system_state()
        col = self.module.detect_collision(self.core.list_rigid_body_instances())
        ref_col = self.ref_module.detect_collision(self.refcore.list_rigid_body_instances())
        all_compare_results = []
        CoR = self.refcore.reference_sim_parameters().CoR
        print(f"CoR {CoR}")
        for col_e, ref_col_e in zip(col, ref_col):
            '''
            Reset system to previous state, so we can inspect the collision
            '''
            # print('restore_system_state')
            self.restore_system_state(rb_state)

            col_e.body1 = ref_col_e.body1
            col_e.body2 = ref_col_e.body2
            col_e.collidingVertex = ref_col_e.collidingVertex
            col_e.collidingTet = ref_col_e.collidingTet
            col_pick = set([col_e])
            ref_col_pick = set([ref_col_e])

            rel_vel_before = self.refcore.compute_relative_velocity([ref_col_e])
            # print(f'rel_vel_before {rel_vel_before}')
            '''
            No need to apply impulse
            '''
            if rel_vel_before < 0:
                continue
            self.core.apply_collision_impulses(col_pick)
            rbi_states, _ = self.backup_system_state()
            '''
            Load the student's output state to refcore
            '''
            self.restore_system_state((rbi_states, rbi_states))
            rel_vel_after = self.refcore.compute_relative_velocity([ref_col_e])
            expect = -CoR * rel_vel_before
            print(f'rel_vel_before {rel_vel_before} rel_vel_after {rel_vel_after} expect {expect}')
            all_compare_results.append(np.allclose(rel_vel_after, expect))
        return all_compare_results

    def sync_with_reference(self):
        for inst_id, ref_inst_id in zip(self.inst_ids, self.ref_inst_ids):
            rbi = self.core.query_rigid_body_instance(inst_id)
            ref_rbi = self.refcore.query_rigid_body_instance(ref_inst_id)
            # print(f'synching {inst_id} <- {ref_inst_id}')
            _sync_rbi(rbi, ref_rbi)

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
4.2 Signed Distance Field
'''
class SignedDistance(TestBase):
    PTS = 7
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        random.seed(bytes.fromhex('d8972bce8d3a81e549af6f74f0b20c5d'))
        for p in get_rbt_models():
            fn = p.stem
            print(f"load {p}")
            birbt = BiRBT(module, str(p))
            for tet_index, tet in enumerate(birbt.tets):
                w = np.zeros(shape=(4), dtype=float)
                rem = 1.0
                for i in range(3):
                    w[i] = random.uniform(0, rem)
                    rem -= w[i]
                w[3] = rem
                coord = birbt.verts[tet[0]] * w[0]
                for i in range(1, 4):
                    coord += birbt.verts[tet[i]] * w[i]
                ret.add_result(birbt.compare_distance(coord, tet_index), name=f'{fn} distance')

class JacobianOfSignedDistance(TestBase):
    PTS = 8
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        random.seed(bytes.fromhex('fefc49157833c480d200598d006d41c2'))
        for p in get_rbt_models():
            fn = p.stem
            birbt = BiRBT(module, str(p))
            for tet_index, _ in enumerate(birbt.tets):
                ret.add_result(birbt.compare_jacobian(tet_index), name=f'{fn} distance jacobian')

'''
End of Signed Distance Field
'''

"""
Generate bicore that has two unit tets that collide
"""

def _update_rbi(rbi, param: RigidBodyParameters):
    if param.density is not None:
        rbi.density = param.density
    rbi.c = param.center
    rbi.theta = param.rotation
    rbi.cvel = param.center_velocity
    rbi.w = param.angular_velocity

def _read_rbi(rbi):
    return RigidBodyParameters(density=rbi.density,
                               center=rbi.c,
                               rotation=rbi.theta,
                               center_velocity=rbi.cvel,
                               angular_velocity=rbi.w)

def _set_rbi_moving_towards(rbi, target, after_t=1.0, from_above=False):
    velocity_dir = -_randunitvec3()
    velocity_mag = random.uniform(0.1, 20.0)
    if from_above:
        velocity_dir[1] = -abs(velocity_dir[1])
    init_c = target - after_t * velocity_mag * velocity_dir
    init_cv = velocity_mag * velocity_dir
    init_rot = _randvec3(randmag=2*math.pi)
    init_w = _randvec3(randmag=2*math.pi)
    # print(f"Sample {init_c} {init_cv} {init_rot} {init_w}")
    param = RigidBodyParameters(density=None,
                                center=init_c,
                                rotation=init_rot,
                                center_velocity=init_cv,
                                angular_velocity=init_w)
    _update_rbi(rbi, param)
    # print(f"rbi: {rbi.c} {rbi.theta} {rbi.cvel} {rbi.w}")
    return param

class ColGen(object):

    def __init__(self, module, ref_module, penalty_force=False, impulse=False):
        bicore = BiCore(module, ref_module)
        refcore = bicore.refcore
        bicore.set_simp("gravityEnabled", False)
        bicore.set_simp("gravityG", 0.0)
        bicore.set_simp("penaltyEnabled", penalty_force)
        if penalty_force:
            bicore.set_simp("penaltyStiffness", 1.0)
        else:
            bicore.set_simp("penaltyStiffness", 0.0)
        bicore.set_simp("impulsesEnabled", impulse)
        bicore.set_simp("NewtonMaxIters", 20)
        bicore.set_simp("NewtonTolerance", 1e-9)

        self.bicore = bicore

        expcore, expsimp = _prepare(ref_module)
        expsimp.penaltyEnabled = False
        expsimp.penaltyStiffness = 0.0
        expsimp.impulsesEnabled = False
        expsimp.gravityEnabled = False
        expsimp.gravityG = 0.0
        expsimp.NewtonMaxIters = 20
        expsimp.NewtonTolerance = 1e-9

        self.expcore = expcore

        self._load_rbi()

    def _load_rbi(self):
        bicore = self.bicore
        expcore = self.expcore

        SCALE = 1.0
        # unit_tet_fn = '../assets/birds/unit_tet.npz'
        unit_tet_fn = '../assets/birds/sphere.npz'
        self.rbt_pair = bicore.add_rbt_from_npz(unit_tet_fn, scale=SCALE)
        _, ref_rbt = self.rbt_pair
        self.com = ref_rbt.com
        param = RigidBodyParameters(density=random.uniform(1, 100),
                                    center=[0.0,0.0,0.0],
                                    rotation=[0.0,0.0,0.0],
                                    center_velocity=[0.0,0.0,0.0],
                                    angular_velocity=[0.0,0.0,0.0])
        bicore.add_instance(self.rbt_pair, param)
        bicore.add_instance(self.rbt_pair, param)
        self.rbi_0, self.ref_rbi_0 = bicore.get_instance_pair_at(0)
        self.rbi_1, self.ref_rbi_1 = bicore.get_instance_pair_at(1)

        exp_0 = _add_rbi(expcore, ref_rbt, param)
        exp_1 = _add_rbi(expcore, ref_rbt, param)
        self.exp_rbi_0 = expcore.query_rigid_body_instance(exp_0)
        self.exp_rbi_1 = expcore.query_rigid_body_instance(exp_1)

    def sample_colpair(self, check_relvel=False):
        bicore = self.bicore
        module = bicore.module
        ref_module = bicore.ref_module
        refcore = bicore.refcore

        expcore = self.expcore
        SHAPE = (3*len(bicore.inst_ids))
        cforce = np.zeros(SHAPE)
        thetaforce = np.zeros(SHAPE)

        TIME = 0.25
        time_step = float(expcore.reference_sim_parameters().timeStep)
        MAXIMUM_STEP = int(TIME / time_step)

        while True:
            """
            Random sample a colliding point
            """
            col_point = _randvec3(lo=-20.0, hi=20.0)
            col_point[1] += 120.0 # Move off the floor, with large margin
            init_0 = _set_rbi_moving_towards(self.exp_rbi_0, col_point, after_t=TIME)
            init_1 = _set_rbi_moving_towards(self.exp_rbi_1, col_point, after_t=TIME)
            okay = False
            for i in range(MAXIMUM_STEP+20):
                expcore.simulate_one_step()
                colset = ref_module.detect_collision([self.exp_rbi_0, self.exp_rbi_1])
                '''
                expcore.compute_penalty_collision_forces(colset, cforce, thetaforce)
                if np.any(cforce) or np.any(thetaforce):
                    okay = True
                    print(f'Colliding at Step {i}, cforce {cforce}, thetaforce {thetaforce}')
                    break
                '''
                if len(colset) == 0:
                    continue
                if not check_relvel:
                    okay = True
                    # print(f'Colliding at Step {i}, size {len(colset)}')
                    # print(f'Relative Velocities {relvels}')
                    break
                else:
                    relvels = expcore.compute_relative_velocity(list(colset))
                    if np.any(relvels > 0.0):
                        okay = True
                        # print(f'Colliding at Step {i}, size {len(colset)}')
                        # print(f'Relative Velocities {relvels}')
                        break
            if not okay:
                print("retrying")
                continue
            '''
            Set both cores at this colliding state.
            and leave
            '''
            # print(f'colliding state {ref_rbi_0.c} {ref_rbi_0.theta}\t{ref_rbi_1.c} {ref_rbi_1.theta} ')
            _sync_rbi(self.ref_rbi_0, self.exp_rbi_0)
            _sync_rbi(self.ref_rbi_1, self.exp_rbi_1)
            bicore.sync_with_reference()

            # colset = ref_module.detect_collision([self.ref_rbi_0, self.ref_rbi_1])
            # refcore.compute_penalty_collision_forces(colset, cforce, thetaforce)
            # print(f'Double check cforce {cforce}, thetaforce {thetaforce}')

            # print(f'Triple check')
            # bicore.compare_penalty_force(compare_cforce=True, compare_theta_force=True)
            # print(f'colliding state {rbi_0.c} {rbi_0.theta}\t{rbi_1.c} {rbi_1.theta} ')
            break

    def sample_colfloor(self, check_relvel=False):
        bicore = self.bicore
        module = bicore.module
        ref_module = bicore.ref_module
        refcore = bicore.refcore

        expcore = self.expcore

        TIME = 0.25
        time_step = float(expcore.reference_sim_parameters().timeStep)
        MAXIMUM_STEP = int(TIME / time_step)
        self.exp_rbi_1.c = np.array([1e4, 1e4, 1e4])
        self.exp_rbi_1.theta = np.array([0, 0, 0])
        self.exp_rbi_1.cvel = np.array([0, 0, 0])
        self.exp_rbi_1.w = np.array([0, 0, 0])

        while True:
            """
            Random sample a colliding point
            """
            col_point = _randvec3(lo=-1.0, hi=20.0)
            col_point[1] = -1.0 # Move off the floor, with large margin
            init_0 = _set_rbi_moving_towards(self.exp_rbi_0, col_point, after_t=TIME, from_above=True)
            okay = False
            for i in range(MAXIMUM_STEP+20):
                # print(f'Simulating {i}')
                expcore.simulate_one_step()
                # print(f'Simulated {i}')
                colset = ref_module.detect_collision([self.exp_rbi_0])
                # print(f'Collision detected {i}')
                if len(colset) == 0:
                    continue
                if not check_relvel:
                    okay = True
                    # print(f'Colliding at Step {i}, size {len(colset)}')
                    # print(f'Relative Velocities {relvels}')
                    break
                else:
                    relvels = expcore.compute_relative_velocity(list(colset))
                    # print(f'RelVel checked {i}')
                    if np.any(relvels > 0.0):
                        okay = True
                        # print(f'Colliding at Step {i}, size {len(colset)}')
                        # print(f'Relative Velocities {relvels}')
                        break
            if not okay:
                print("Not colliding, retrying")
                continue
            '''
            Set both cores at this colliding state.
            and leave
            '''
            # print(f'colliding state {ref_rbi_0.c} {ref_rbi_0.theta}\t{ref_rbi_1.c} {ref_rbi_1.theta} ')
            _sync_rbi(self.ref_rbi_0, self.exp_rbi_0)
            _sync_rbi(self.ref_rbi_1, self.exp_rbi_1)
            bicore.sync_with_reference()

            # colset = ref_module.detect_collision([self.ref_rbi_0, self.ref_rbi_1])
            # refcore.compute_penalty_collision_forces(colset, cforce, thetaforce)
            # print(f'Double check cforce {cforce}, thetaforce {thetaforce}')

            # print(f'Triple check')
            # bicore.compare_penalty_force(compare_cforce=True, compare_theta_force=True)
            # print(f'colliding state {rbi_0.c} {rbi_0.theta}\t{rbi_1.c} {rbi_1.theta} ')
            break

PENALTY_METHOD_NSAMPLE = 10

'''
4.4 Penalty Method
'''
class CPenaltyForceForBody(TestBase):
    PTS = 15
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        random.seed(bytes.fromhex('9905c97e8df7a829'))
        ref_module = importlib.import_module('pypsim_ref.bird2')
        colgen = ColGen(module, ref_module, penalty_force=True, impulse=False)
        for i in range(PENALTY_METHOD_NSAMPLE):
            colgen.sample_colpair()
            compare_results = colgen.bicore.compare_penalty_force(compare_cforce=True, compare_theta_force=False)
            for result in compare_results:
                ret.add_result(result, name='Body vs Body C Force')
            # print('-------------------------------')

class ThetaPenaltyForceForBody(TestBase):
    PTS = 15
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        random.seed(bytes.fromhex('9905c97e8df7a829'))
        ref_module = importlib.import_module('pypsim_ref.bird2')
        colgen = ColGen(module, ref_module, penalty_force=True, impulse=False)
        for i in range(PENALTY_METHOD_NSAMPLE):
            colgen.sample_colpair()
            compare_results = colgen.bicore.compare_penalty_force(compare_cforce=False,
                                                                  compare_theta_force=True)
            for result in compare_results:
                ret.add_result(result, name='Body vs Body Theta Force')
            # print('-------------------------------')

class CPenaltyForceForFloor(TestBase):
    PTS = 7.5
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        random.seed(bytes.fromhex('1cc853fe91743d79'))
        ref_module = importlib.import_module('pypsim_ref.bird2')
        colgen = ColGen(module, ref_module, penalty_force=True, impulse=False)
        for i in range(PENALTY_METHOD_NSAMPLE):
            colgen.sample_colfloor()
            compare_result = colgen.bicore.compare_penalty_force(compare_cforce=True,
                                                                 compare_theta_force=False)
            ret.add_result(compare_result, name='Body vs Floor C Force')


class ThetaPenaltyForceForFloor(TestBase):
    PTS = 7.5
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        random.seed(bytes.fromhex('1cc853fe91743d79'))
        ref_module = importlib.import_module('pypsim_ref.bird2')
        colgen = ColGen(module, ref_module, penalty_force=True, impulse=False)
        for i in range(PENALTY_METHOD_NSAMPLE):
            colgen.sample_colfloor()
            compare_result = colgen.bicore.compare_penalty_force(compare_cforce=True,
                                                                 compare_theta_force=False)
            ret.add_result(compare_result, name='Body vs Floor Theta Force')


IMPULSE_METHOD_NSAMPLE = 10

'''
4.5 Collision Impulses
'''
class RelativeVelocity(TestBase):
    PTS = 5
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        # random.seed(bytes.fromhex('cb953dbc0f6ba4e7'))
        '''
        Free points
        Unfortunately we do not have enough interface for this step.
        '''
        ret.add_result(True)

class BestPair(TestBase):
    PTS = 5
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        # random.seed(bytes.fromhex('440a8dbf637607d8'))
        '''
        Free points
        Unfortunately we do not have enough interface for this step.
        '''
        ret.add_result(True)

class BodyBodyImpulse(TestBase):
    PTS = 20
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        random.seed(bytes.fromhex('775af5e53329f070'))
        ref_module = importlib.import_module('pypsim_ref.bird2')
        colgen = ColGen(module, ref_module, penalty_force=False, impulse=True)
        for i in range(IMPULSE_METHOD_NSAMPLE):
            colgen.bicore.set_simp('CoR', random.uniform(0.0, 1.0))
            colgen.sample_colpair(check_relvel=True)
            compare_results = colgen.bicore.compare_impulse()
            for result in compare_results:
                ret.add_result(result, name='Body vs Body Impulses')

class BodyFloorImpulse(TestBase):
    PTS = 10
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        random.seed(bytes.fromhex('25b9ffde6b582b1b'))
        ref_module = importlib.import_module('pypsim_ref.bird2')
        colgen = ColGen(module, ref_module, penalty_force=False, impulse=True)
        for i in range(IMPULSE_METHOD_NSAMPLE):
            colgen.bicore.set_simp('CoR', random.uniform(0.0, 1.0))
            colgen.sample_colfloor(check_relvel=True)
            compare_results = colgen.bicore.compare_impulse()
            for result in compare_results:
                ret.add_result(result, name='Body vs Floor Impulses')


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

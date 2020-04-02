import inspect
import numpy as np
import math
import random
from base import *
import util

def create_core(module):
    c = module.GooCore()
    c.init_simulation()
    return c, c.reference_sim_parameters()

def _prepare(module):
    core, simp = create_core(module)
    return core, simp

def _ref_prepare():
    try:
        from pypsim_ref.goo2 import GooCore as RefGooCore
    except ModuleNotFoundError as e:
        print(e)
        print("This script should be executed under bin/. Example")
        print("psim/bin$ ../grading/psim_grade.py --case goo1 .....")
        exit()
    c = RefGooCore()
    c.init_simulation()
    return c, c.reference_sim_parameters()

def _particle_energy(core, pid):
    p, valid = core.query_particle(pid)
    g = core.reference_sim_parameters().gravityG
    return 0.5 * np.inner(p.vel, p.vel) - g * (p.pos[1] + 2.0)

class BiCore(object):
    def __init__(self, core, refcore):
        self.core = core
        self.refcore = refcore
        self.simp = self.core.reference_sim_parameters()
        self.refsimp = self.refcore.reference_sim_parameters()

    def set_simp(self, name, val):
        setattr(self.simp, name, val)
        setattr(self.refsimp, name, val)

    def set_ch(self, ch_name):
        handler = getattr(self.simp, ch_name)
        self.simp.constraintHandling = handler
        refhandler = getattr(self.refsimp, ch_name)
        self.refsimp.constraintHandling = refhandler

    def set_ct(self, ct_name):
        conn = getattr(self.simp, ct_name)
        self.simp.connectorType = conn
        refconn = getattr(self.refsimp, ct_name)
        self.refsimp.connectorType = refconn

    def add_particle(self, x, y):
        pid = self.core.add_particle(x, y)
        refpid = self.refcore.add_particle(x, y)
        return (pid, refpid)

    def simulate_one_step(self):
        self.core.simulate_one_step()
        self.refcore.simulate_one_step()

    def compare(self, pid_pair, rtol=1e-4):
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

    '''
    chamfer_matching is a dictory from pid to refpid
    '''
    def chamfer_compare(self, pid_pair, chamfer_matching=None, rtol=None):
        if chamfer_matching is None:
            pid_vec, refpid_vec = pid_pair
            queries = [self.core.query_particle(pid) for pid in pid_vec]
            refqueries = [self.refcore.query_particle(refpid) for refpid in refpid_vec]
            def query_to_pos(queries):
                all_pos = np.empty(shape=(len(queries), 2), dtype=np.float64)
                for i in range(len(queries)):
                    all_pos[i,:] = queries[i][0].pos
                return all_pos
            all_pos = query_to_pos(queries)
            all_refpos = query_to_pos(refqueries)
            chamfer_matching = {}
            for i, pos in enumerate(all_pos):
                distances = np.linalg.norm(all_refpos - pos, axis=1)
                # print(distances)
                close_i = np.nanargmin(distances)
                chamfer_matching[pid_vec[i]] = refpid_vec[close_i]
            return chamfer_matching
        pid_vec, refpid_vec = pid_pair
        for i in range(len(pid_vec)):
            refpid_vec[i] = chamfer_matching[pid_vec[i]]
        if rtol is None:
            return self.compare((pid_vec, refpid_vec))
        else:
            return self.compare((pid_vec, refpid_vec), rtol=rtol)

    def reset(self):
        self.core.init_simulation()
        self.refcore.init_simulation()

def _bicore_add_pendulum(bicore):
    bicore.set_simp('gravityG', -9.8)
    bicore.set_simp('gravityEnabled', True)
    bicore.set_simp('dampingEnabled', False)
    bicore.set_simp('dampingStiffness', 0.0)

    bicore.set_simp('maxSpringDist', 10.0)
    bicore.set_simp('particleFixed', True)
    bicore.set_simp('particleMass', 10.0)
    fixed_pid_pair = bicore.add_particle(0.0, 0.0)
    bicore.set_simp('particleFixed', False)
    moving_pid_pair = bicore.add_particle(0.5, 0.0)
    return fixed_pid_pair, moving_pid_pair

def _prepare_bicore(module):
    core,_ = _prepare(module)
    refcore,_ = _ref_prepare()
    bicore = BiCore(core, refcore)
    """
    We do NOT test damping in Goo 2
    """
    bicore.set_simp('dampingEnabled', False)
    return bicore

'''
4.1 Ridig Rods

Free credits
'''
class RigidRodsBasics(TestBase):
    PTS = 5
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        """
        Automatically awarded
        """
        ret.add_result(True)


'''
4.1.1 Penalty Method

Create a pendulum and verify first 100 steps
'''
class PenaltyMethod(TestBase):
    PTS = 5
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        core, simp = _prepare(module)
        simp.gravityG = -9.8
        simp.gravityEnabled = True
        simp.dampingEnabled = False
        simp.dampingStiffness = 0.0
        simp.maxSpringDist = 10.0
        simp.particleFixed = True
        simp.particleMass = 10.0
        simp.timeStep = 1e-4
        fixed_pid = list(core.add_particle(0.0, 0.0))[0]
        simp.particleFixed = False
        moving_pid = list(core.add_particle(0.5, 0.0))[0]
        for i in range(100):
            core.simulate_one_step()
            p, valid = core.query_particle(moving_pid)
            dist = np.linalg.norm(p.pos - np.array([0.0, 0.0]))
            # print(f"{p.pos} {dist}")
            ret.add_result(np.allclose(dist, 0.5), name=f"Penalty Method Iteration {i:04d}")


        '''
        bicore = _prepare_bicore(module)
        bicore.set_ct('CT_RIGIDROD')
        bicore.set_ch('CH_PENALTY')
        fixed_pid_pair, moving_pid_pair = _bicore_add_pendulum(bicore)
        for i in range(100):
            bicore.simulate_one_step()
            ret.add_result(bicore.compare(moving_pid_pair), name=f"Penalty Method Iteration {i:04d}")
        '''


'''
4.1.2 Step and Project

Two cases
1. Create a pendulum and verify first 100 steps
2. Create a rigid cage attached to a fixed particle and verify first 100 steps
'''
class StepAndProject(TestBase):
    PTS = 25
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        bicore = _prepare_bicore(module)
        bicore.set_ct('CT_RIGIDROD')
        bicore.set_ch('CH_STEPPROJECT')
        fixed_pid_pair, moving_pid_pair = _bicore_add_pendulum(bicore)
        for i in range(100):
            bicore.simulate_one_step()
            ret.add_result(bicore.compare(moving_pid_pair), weight=6, name=f"Step And Project Test A Iteration {i:04d}")
        bicore = _prepare_bicore(module)
        bicore.set_ct('CT_RIGIDROD')
        bicore.set_ch('CH_STEPPROJECT')
        fixed_pid_pair, moving_pid_pair = _bicore_add_pendulum(bicore)
        more_moving_pid_pairs = []
        X = [0.25,  0.17, -0.92, -1.13, 0.71]
        Y = [0.25, -0.35, -0.38,  0.84, 1.92]
        for i in range(5):
            more_moving_pid_pairs.append(bicore.add_particle(X[i], Y[i]))
        all_moving_pid_pairs = [moving_pid_pair] + more_moving_pid_pairs
        for i in range(100):
            bicore.simulate_one_step()
            for j, pair in enumerate(all_moving_pid_pairs):
                ret.add_result(bicore.compare(pair),
                        name=f"Step And Project Test B Iteration {i:04d} Pair {j:01d}")


'''
4.1.3 Constrained Lagrangian

The same cases from step and project
'''
class ConstrainedLagrangian(TestBase):
    PTS = 15
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        bicore = _prepare_bicore(module)
        bicore.set_ct('CT_RIGIDROD')
        bicore.set_ch('CH_LAGRANGEMULT')
        fixed_pid_pair, moving_pid_pair = _bicore_add_pendulum(bicore)
        for i in range(100):
            bicore.simulate_one_step()
            ret.add_result(bicore.compare(moving_pid_pair), weight=6, name=f"Constrained Lagrangian Test A Iteration {i:04d}")
        bicore = _prepare_bicore(module)
        bicore.set_ct('CT_RIGIDROD')
        bicore.set_ch('CH_LAGRANGEMULT')
        fixed_pid_pair, moving_pid_pair = _bicore_add_pendulum(bicore)
        more_moving_pid_pairs = []
        X = [0.25,  0.17, -0.92, -1.13, 0.71]
        Y = [0.25, -0.35, -0.38,  0.84, 1.92]
        for i in range(5):
            more_moving_pid_pairs.append(bicore.add_particle(X[i], Y[i]))
        all_moving_pid_pairs = [moving_pid_pair] + more_moving_pid_pairs
        for i in range(100):
            bicore.simulate_one_step()
            for j, pair in enumerate(all_moving_pid_pairs):
                ret.add_result(bicore.compare(pair),
                        name=f"Constrained Lagrangian Test B Iteration {i:04d} Pair {j:01d}")


'''
4.2 Flexible Rods

Two cases:
1. Verify the number of created particles (including inert)
2. Verify the particle is unsnappable by setting a very low maxSpringStrain

Spring mass is given as free credit.
'''
class FlexibleRodsBasics(TestBase):
    PTS = 5
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        core, simp = _prepare(module)
        simp.particleFixed = True
        _ = core.add_particle(0.0, 0.0)
        simp.particleFixed = False
        simp.particleMass = 1e9
        simp.constraintHandling = simp.CH_STEPPROJECT
        simp.connectorType = simp.CT_FLEXROD
        simp.rodSegments = 10
        raw_pids = list(core.add_particle(0.0, -1e-1))
        pids = raw_pids[1:] + raw_pids[0:1]
        ret.add_result(len(pids) == simp.rodSegments, weight=2, name='Inert')
        ret.add_result(True, weight=1, name='Spring mass (auto)')
        simp.bendingEnabled = False
        simp.maxSpringStrain = 1e-4
        simp.rodStretchingStiffness = 0.0
        simp.gravityG = -1
        simp.gravityEnabled = True
        unsnappable = True
        for i in range(1000):
            core.simulate_one_step()
            for pid in pids:
                _, valid = core.query_particle(pid)
                if not valid:
                    unsnappable = False
                    break
            if not unsnappable:
                break
        ret.add_result(unsnappable, weight=2, name='Unsnappable')


'''
4.2.2 Stretching and Bending (Part 1)

Two cases:
1. inert is set correctly.
2. distances between particles are correct
'''
class StretchingAndBending_1(TestBase):
    PTS = 5
    RUBIRIC_NAME = 'Stretching and Bending 1: Inerts'

    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        core, simp = _prepare(module)
        simp.bendingEnabled = True
        simp.particleFixed = True
        _ = core.add_particle(0.0, 0.0)
        simp.particleFixed = False
        simp.particleMass = 1e9
        simp.maxSpringDist = 10.0
        simp.constraintHandling = simp.CH_STEPPROJECT
        simp.connectorType = simp.CT_FLEXROD
        TOTAL_LENGTH = 1.0
        simp.rodSegments = 10
        raw_pids = list(core.add_particle(0.0, TOTAL_LENGTH))
        pids = raw_pids[1:] + raw_pids[0:1]
        particles = []
        valids = []
        for pid in pids:
            p, v = core.query_particle(pid)
            particles.append(p)
            valids.append(v)
        if len(particles) == 1:
            ret.add_result(False, name='No inert particle returned')
            return
        for p in particles[:-1]:
            ret.add_result(p.inert, name='Inert particle')
        distances = []
        for pp,pn in zip(particles[:-1], particles[1:]):
            distances.append(np.linalg.norm(pp.pos - pn.pos))
        distances = np.array(distances) - (TOTAL_LENGTH/simp.rodSegments)
        for d in distances:
            ret.add_result(np.allclose([d], 0.0))


'''
4.2.2 Stretching and Bending (Part 2)

Free credits, since it cannot be evaluated without Part 3
'''
class StretchingAndBending_2(TestBase):
    PTS = 10
    RUBIRIC_NAME = 'Stretching and Bending 2: Hinge'

    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        """
        Automatically awarded. Unless Bending 3 is zero
        """
        ret.add_result(True, name='Free pts')


'''
4.2.2 Stretching and Bending (Part 3)

Four fixed particles support one free particle.
The configuration looks like:

     __o_____
    / /  \   \
---x--x--x---x---

The first 100 steps are verified.
'''
class StretchingAndBending_3(TestBase):
    PTS = 20
    RUBIRIC_NAME = 'Stretching and Bending 3: Elastic'

    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        bicore = _prepare_bicore(module)
        bicore.set_ch('CH_STEPPROJECT')
        bicore.set_simp('bendingEnabled', True)
        bicore.set_simp('particleFixed', True)
        bicore.set_simp('maxSpringDist', 0.0)
        bicore.set_ct('CT_RIGIDROD')
        fix1 = bicore.add_particle(-0.1, 0.0)
        fix2 = bicore.add_particle(-0.3, 0.0)
        fix3 = bicore.add_particle(0.27, 0.05)
        fix4 = bicore.add_particle(0.91, 0.12)
        bicore.set_simp('particleFixed', False)
        bicore.set_simp('particleMass', 100.0)
        bicore.set_simp('gravityG', -9.8)
        bicore.set_simp('gravityEnabled', True)
        bicore.set_simp('rodSegments', 10)
        bicore.set_ct('CT_FLEXROD')
        bicore.set_simp('maxSpringDist', 10.0)
        moving = bicore.add_particle(0.0, 1.125)
        '''
        print(moving)
        pid_vec, refpid_vec = moving
        for pid, refpid in zip(pid_vec, refpid_vec):
            p, valid = bicore.core.query_particle(pid)
            refp, refvalid = bicore.refcore.query_particle(refpid)
            print(f"{p.pos} {refp.pos}")
        '''
        ret.add_result(bicore.compare(moving), name=f'Iteration -1')
        chamfer_matching = bicore.chamfer_compare(moving)
        # print(chamfer_matching)
        # assert bicore.chamfer_compare(moving, chamfer_matching=chamfer_matching)
        for i in range(100):
            bicore.simulate_one_step()
            ret.add_result(bicore.chamfer_compare(moving, rtol=1e-3), name=f'Iteration {i:04d}')


'''
4.2.2 Stretching and Bending (Part 4)

Create two saws to cut the first inert and the connection between 5th and 6th (0-indexed) particle.
'''
class StretchingAndBending_4(TestBase):
    PTS = 10
    RUBIRIC_NAME = 'Stretching and Bending 4: vs Saws'

    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        """
        As usual, a flex rod of 10 segments
        """
        core, simp = _prepare(module)
        simp.particleFixed = True
        _ = core.add_particle(0.0, 0.0)
        simp.particleFixed = False
        simp.particleMass = 1e9
        simp.maxSpringDist = 10.0
        simp.constraintHandling = simp.CH_STEPPROJECT
        simp.connectorType = simp.CT_FLEXROD
        TOTAL_LENGTH = 1.0
        simp.rodSegments = 10
        raw_pids = list(core.add_particle(0.0, TOTAL_LENGTH))
        pids = raw_pids[1:] + raw_pids[0:1]
        particles = []
        valids = []
        for pid in pids:
            p, v = core.query_particle(pid)
            particles.append(p)
            valids.append(v)
        if len(particles) == 1:
            ret.add_result(False, name='No inert particle returned')
            return
        simp.sawRadius = 1e-3
        sp_0 = particles[0].pos
        core.add_saw(sp_0[0], sp_0[1])
        sp_1 = 0.5 * (particles[5].pos + particles[6].pos)
        core.add_saw(sp_1[0], sp_1[1])

        simp.gravityG = 0.0
        core.simulate_one_step()
        _, valid = core.query_particle(pids[0])
        ret.add_result(not valid, name='Saw vs inert 1')
        conn = core.query_connectivity(pids[:-1], pids[1:])
        # print(f'conn {conn}')
        ret.add_result(not conn[0], name='Saw vs inert 2')
        ret.add_result(conn[1], name='Saw vs inert 3')
        ret.add_result(conn[2], name='Saw vs inert 4')
        ret.add_result(conn[3], name='Saw vs inert 5')
        ret.add_result(conn[4], name='Saw vs inert 6')
        ret.add_result(not conn[5], name='Saw vs inert 7')
        ret.add_result(conn[6], name='Saw vs inert 8')
        ret.add_result(conn[7], name='Saw vs inert 9')
        ret.add_result(conn[8], name='Saw vs inert 10')


'''
Auxiliary functions
'''
def get_test_objects():
    test_objects = []
    g = globals().copy()
    for name, obj in g.items():
        # print(f"{name}")
        if not inspect.isclass(obj):
            continue
        if not issubclass(obj, TestBase) or obj == TestBase:
            continue
        test_objects.append(obj())
    total_pts = np.sum([t.report_total_points() for t in test_objects])
    assert total_pts == 100, f'Total points {total_pts} != 100'
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

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
    from pypsim_ref.goo1 import GooCore as RefGooCore
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
        self.set_int('TI_EXPLICIT_EULER') # Easiest integrator

    def set_simp(self, name, val):
        setattr(self.simp, name, val)
        setattr(self.refsimp, name, val)

    def set_int(self, integrator_name):
        integrator = getattr(self.simp, integrator_name)
        self.simp.integrator = integrator
        refintegrator = getattr(self.refsimp, integrator_name)
        self.refsimp.integrator = refintegrator

    def add_particle(self, x, y):
        pid = self.core.add_particle(x, y)
        refpid = self.refcore.add_particle(x, y)
        return (pid, refpid)

    def simulate_one_step(self):
        self.core.simulate_one_step()
        self.refcore.simulate_one_step()

    def compare(self, pid_pair):
        pid, refpid = pid_pair
        p, valid = self.core.query_particle(pid)
        refp, refvalid = self.refcore.query_particle(refpid)
        if valid != refvalid:
            print(f"valid ({valid}) != refvalid ({refvalid})")
            return False
        '''
        if not np.allclose(p.mass, refp.mass):
            print(f"mass ({p.mass}) != refmass ({refp.mass})")
            return False
        '''
        if not np.allclose(p.pos, refp.pos, rtol=1e-4):
            print(f"pos ({p.pos}) != refpos ({refp.pos})")
            return False
        if not np.allclose(p.vel, refp.vel, rtol=1e-4):
            print(f"vel ({p.vel}) != refvel ({refp.vel})")
            return False
        return True

    def reset(self):
        self.core.init_simulation()
        self.refcore.init_simulation()

def _prepare_bicore(module):
    core,_ = _prepare(module)
    refcore,_ = _ref_prepare()
    return BiCore(core, refcore)

class BasicSpringUI(TestBase):
    PTS = 10
    RUBIRIC_NAME = 'Basic Spring UI'
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        core, simp = _prepare(module)
        for i in range(10):
            mass = random.uniform(1e-3, 1e3)
            simp.particleMass = mass
            pid = core.add_particle(1.0, 1.0)
            p, valid = core.query_particle(pid)
            ret.add_result(valid, name="particle was not added")
            ret.add_result(p.mass == mass, name="mass mismatch")

class ConfigurationVectorBuilding(TestBase):
    PTS = 5
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        """
        Automaticall awarded
        """
        ret.add_result(True)

class FdFGravity(TestBase):
    PTS = 5
    RUBIRIC_NAME = 'FdF Gravity'

    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        bicore = _prepare_bicore(module)
        bicore.set_simp('gravityG', 100.0)
        pid_pair = bicore.add_particle(0.0, 1.5)
        for i in range(25):
            bicore.simulate_one_step()
            ret.add_result(bicore.compare(pid_pair), name=f'Iteration {i}')

class FdFSpring(TestBase):
    PTS = 15
    RUBIRIC_NAME = 'FdF Spring'

    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        bicore = _prepare_bicore(module)
        bicore.set_simp('gravityG', -1.0)
        bicore.set_simp('dampingEnabled', False)
        bicore.set_simp('maxSpringStrain', 10000.0)
        bicore.set_simp('maxSpringDist', 10.0)
        pid_pair_1 = bicore.add_particle(0.0, 0.5)
        bicore.set_simp('particleFixed', False)
        bicore.set_simp('springStiffness', 1e2)
        pid_pair_2 = bicore.add_particle(0.5, 0.0)
        for i in range(100):
            bicore.simulate_one_step()
            ret.add_result(bicore.compare(pid_pair_1), name=f'Iteration {i} pair 1')
            ret.add_result(bicore.compare(pid_pair_2), name=f'Iteration {i} pair 2')

        bicore.reset()

        bicore.set_simp('gravityG', -1.0)
        bicore.set_simp('dampingEnabled', False)
        bicore.set_simp('maxSpringStrain', 10000.0)
        bicore.set_simp('maxSpringDist', 0.8)
        bicore.set_simp('springStiffness', 1.0)
        bicore.set_simp('particleFixed', True)
        pid_pair_1 = bicore.add_particle(0.0, 0.0)
        bicore.set_simp('particleFixed', False)
        pid_pair_2 = bicore.add_particle(0.0, 0.4)
        pid_pair_3 = bicore.add_particle(0.4, 0.4)
        pid_pair_4 = bicore.add_particle(0.4, 0.0)
        for i in range(25):
            bicore.simulate_one_step()
            ret.add_result(bicore.compare(pid_pair_1), name=f'Iteration {i} pair 1')
            ret.add_result(bicore.compare(pid_pair_2), name=f'Iteration {i} pair 2')

class MassMatrix(TestBase):
    PTS = 2
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        """
        Automaticall awarded
        """
        ret.add_result(True)

def _test_int(bicore, ret):
    bicore.set_simp('gravityG', -1.0)
    bicore.set_simp('dampingEnabled', False)
    bicore.set_simp('maxSpringStrain', 10000.0)
    bicore.set_simp('maxSpringDist', 0.8)
    bicore.set_simp('springStiffness', 1.0)
    bicore.set_simp('particleFixed', True)
    pid_pairs = []
    pid_pairs.append(bicore.add_particle(0.0, 0.0))
    bicore.set_simp('particleFixed', False)
    pid_pairs.append(bicore.add_particle(0.0, 0.4))
    pid_pairs.append(bicore.add_particle(0.4, 0.4))
    pid_pairs.append(bicore.add_particle(0.4, 0.0))
    for i in range(25):
        bicore.simulate_one_step()
        for k, p in enumerate(pid_pairs):
            ret.add_result(bicore.compare(p), name=f'Iteration {i} pid pair {k}')

class ExplicitEuler(TestBase):
    PTS = 5
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        bicore = _prepare_bicore(module)
        _test_int(bicore, ret)

class VelocityVerlet(TestBase):
    PTS = 5
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        bicore = _prepare_bicore(module)
        bicore.set_int('TI_VELOCITY_VERLET')
        _test_int(bicore, ret)

class ImplicitEuler(TestBase):
    PTS = 10
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        bicore = _prepare_bicore(module)
        bicore.set_int('TI_IMPLICIT_EULER')
        _test_int(bicore, ret)

class ImplicitMidpoint(TestBase):
    PTS = 10
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        bicore = _prepare_bicore(module)
        bicore.set_int('TI_IMPLICIT_MIDPOINT')
        _test_int(bicore, ret)

class FixedDOFs(TestBase):
    PTS = 2
    RUBIRIC_NAME = 'Fixed DOFs'

    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        bicore = _prepare_bicore(module)
        bicore.set_simp('gravityG', 100.0)
        bicore.set_simp('particleFixed', True)
        bicore.set_simp('dampingEnabled', False)
        fixed_pid_pair = bicore.add_particle(0.0, 1.5)
        for i in range(25):
            bicore.simulate_one_step()
            ret.add_result(bicore.compare(fixed_pid_pair), name=f'Single Particle {i}')

        '''
        A particle with large mass should not move the fixed particle
        '''
        bicore.reset()
        bicore.set_simp('gravityG', 100.0)
        bicore.set_simp('particleFixed', True)
        bicore.set_simp('dampingEnabled', False)
        fixed_pid_pair = bicore.add_particle(0.0, 0.0)
        bicore.set_simp('particleFixed', False)
        bicore.set_simp('particleMass', 1e9)
        bicore.set_simp('springStiffness', 1e6)
        bicore.set_simp('maxSpringDist', 10.0)
        moving_pid_pair = bicore.add_particle(1.0, 0.0)

        for i in range(25):
            bicore.simulate_one_step()
            ret.add_result(bicore.compare(fixed_pid_pair), name=f'Double Particle {i}')

class ViscousDamping(TestBase):
    PTS = 10
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        bicore = _prepare_bicore(module)
        bicore.set_int('TI_VELOCITY_VERLET')
        bicore.set_simp('gravityG', -100.0)
        bicore.set_simp('dampingEnabled', True)
        bicore.set_simp('dampingStiffness', 1e3)
        bicore.set_simp('maxSpringStrain', 10000.0)
        bicore.set_simp('maxSpringDist', 10.0)
        bicore.set_simp('springStiffness', 100.0)

        bicore.set_simp('particleFixed', True)
        fixed_pid_pair = bicore.add_particle(0.0, 0.0)
        bicore.set_simp('particleFixed', False)
        bicore.set_simp('particleMass', 1.0)
        bicore.set_simp('maxSpringDist', 10.0)
        moving_pid_pair = bicore.add_particle(0.5, 0.0)

        for i in range(100):
            bicore.simulate_one_step()
            ret.add_result(bicore.compare(moving_pid_pair), name=f'Iteration {i}')
            '''
            print(f'Energy {_particle_energy(bicore.core, moving_pid_pair[0])}', end='\t')
            print(f'RefEnergy {_particle_energy(bicore.refcore, moving_pid_pair[1])}')
            '''

class FloorForce(TestBase):
    PTS = 5
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        TOLERANCE = -1.0
        simp = module.SimParameters
        # all_ints = [simp.TI_EXPLICIT_EULER, simp.TI_IMPLICIT_EULER, simp.TI_IMPLICIT_MIDPOINT, simp.TI_VELOCITY_VERLET]
        all_ints = [simp.TI_VELOCITY_VERLET]
        for ti in all_ints:
            core, simp = _prepare(module)
            simp.integrator = ti
            simp.gravityG = 0.0
            simp.particleMass = 1e9
            simp.floorEnabled = True

            pid = core.add_particle(random.uniform(-1.0, 1.0), random.uniform(-0.5, 1.0))
            p, valid = core.query_particle(pid)
            init_pos = np.copy(p.pos)
            no_functioning_floor = True
            for i in range(10):
                core.simulate_one_step()
                p, valid = core.query_particle(pid)
                if not np.allclose(init_pos, p.pos):
                    no_functioning_floor = False
                    break
            ret.add_result(no_functioning_floor, name=f"{ti}: Floor should do nothing to particles whose y >= -0.5")

            core.init_simulation()
            simp.gravityG = -9.8
            simp.particleMass = 100
            simp.floorEnabled = True
            pid = core.add_particle(0.0, 1.0)
            blocking_the_particle = True
            for i in range(10000):
                core.simulate_one_step()
                p, valid = core.query_particle(pid)
                # print(p.pos)
                if not valid or p.pos[1] < TOLERANCE:
                    blocking_the_particle = False
            ret.add_result(blocking_the_particle, name=f"{ti}: Floor should prevent the particle from hitting y = -1.0")

            core.init_simulation()
            simp.gravityG = 0.0
            simp.particleMass = 100
            simp.dampingEnabled = False
            pid = core.add_particle(0.0, -0.5 - 1e-2)
            bumping_the_particle = False
            for i in range(100):
                # print(p.pos)
                if p.pos[1] > -0.5:
                    bumping_the_particle = True
                    break
            ret.add_result(bumping_the_particle, name=f"{ti}: Floor should bump the particle whose y < -0.5")


class SpringSnapping(TestBase):
    PTS = 5

    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        core, simp = _prepare(module)
        simp.integrator = simp.TI_IMPLICIT_EULER
        simp.gravityG = -9.8
        simp.maxSpringDist = 0.25
        simp.floorEnabled = False
        simp.dampingEnabled = False
        for i in range(100):
            core.init_simulation()
            strain = 0.05 * i
            simp.maxSpringStrain = strain
            simp.particleFixed = True
            pid_fix = core.add_particle(0.0, 1.0)
            simp.particleFixed = False
            pid_moving = core.add_particle(0.2, 1.0)
            for j in range(1000):
                core.simulate_one_step()
                p, valid = core.query_particle(pid_moving)
                # print(f'{valid} {p.pos}')
            p, valid = core.query_particle(pid_moving)
            # print(f'{strain} {valid}')
            if strain < 0.3:
                ret.add_result(valid == False, name="With small strain spring should break")
            else:
                ret.add_result(valid == True, name="With large strain spring should hold")

class Saw(TestBase):
    PTS = 10

    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        core, simp = _prepare(module)
        r = simp.sawRadius
        core.add_saw(0.0, 0.0)
        pids_1 = []
        pids_2 = []
        for i in range(1000):
            x = random.uniform(-r, r)
            y = random.uniform(-r, r)
            if np.linalg.norm([x,y]) <= r:
                pids_1.append(core.add_particle(x,y))
            else:
                pids_2.append(core.add_particle(x,y))
        core.simulate_one_step()
        valid = False
        for pid in pids_1:
            _, valid = core.query_particle(pid)
            if valid != False:
                break
        ret.add_result(valid == False, name='particle should be sawed')

        valid = True
        for pid in pids_2:
            _, valid = core.query_particle(pid)
        ret.add_result(valid == True, name='particle should not be sawed')

        core, simp = _prepare(module)
        simp.gravityG = -9.8
        sr = simp.sawRadius = 1e-4
        simp.maxSpringDist = 0.25
        simp.maxSpringStrain = 1000.0
        simp.integrator = simp.TI_IMPLICIT_EULER
        simp.dampingEnabled = False
        simp.floorEnabled = False

        ox = 0.0
        oy = 0.5
        for K in range(10):
            core.init_simulation()
            """
            Add Pendulum
            """
            simp.particleFixed = True
            pid_fix1 = core.add_particle(ox, oy)
            theta = random.uniform(0.0, 0.25 * math.pi)
            orth_x = math.cos(theta + 0.5 * math.pi)
            orth_y = math.sin(theta + 0.5 * math.pi)
            pp_x = math.cos(theta)
            pp_y = math.sin(theta)
            dx = pp_x * simp.maxSpringDist * 0.6
            dy = pp_y * simp.maxSpringDist * 0.6
            simp.particleFixed = False
            mx = ox + dx
            my = oy + dy
            pid_moving = core.add_particle(mx, my)
            # N = 100
            N = 2
            for i in range(1, N):
                tau = float(i) / N
                sx = (1.0 - tau) * ox + tau * mx
                sy = (1.0 - tau) * oy + tau * my
                sx += sr * (1+1e-5) * orth_x
                sy += sr * (1+1e-5) * orth_y
                core.add_saw(sx, sy)
            sx = mx + (1+1e-3) * sr * pp_x
            sy = my + (1+1e-3) * sr * pp_y
            core.add_saw(sx, sy)
            ok = True
            for i in range(1000):
                core.simulate_one_step()
                p, valid = core.query_particle(pid_moving)
                if valid == False or p.pos[1] <= -0.5:
                    ok = False
                    break
            ret.add_result(ok == True, name='This particle should still be a part of the pendulum')

        for K in range(10):
            core.init_simulation()
            """
            Add Pendulum
            """
            simp.particleFixed = True
            pid_fix1 = core.add_particle(ox, oy)
            theta = random.uniform(0.0, 0.25 * math.pi)
            orth_x = math.sin(theta)
            orth_y = math.cos(theta)
            pp_x = math.cos(theta)
            pp_y = math.sin(theta)
            dx = pp_x * simp.maxSpringDist * 0.6
            dy = pp_y * simp.maxSpringDist * 0.6
            simp.particleFixed = False
            mx = ox + dx
            my = oy + dy
            pid_moving = core.add_particle(mx, my)
            tau = 0.5
            sx = (1.0 - tau) * ox + tau * mx
            sy = (1.0 - tau) * oy + tau * my
            sx -= sr * (1-1e-5) * orth_x
            sy -= sr * (1-1e-5) * orth_y
            core.add_saw(sx, sy)
            for i in range(1000):
                core.simulate_one_step()
                p, valid = core.query_particle(pid_moving)
            ret.add_result(not valid, name='The connector of this particle should be sawed')

class Miscellaneous(TestBase):
    PTS = 1
    def __init__(self):
        super().__init__()

    def do_test(self, module, ret):
        core, simp = _prepare(module)
        pids = []
        for i in range(10):
            x = random.uniform(-1, 1)
            y = random.uniform(-1, 1)
            pids.append(core.add_particle(x,y))
        core.init_simulation()
        valid = False
        for pid in pids:
            _, valid = core.query_particle(pid)
            if valid == True:
                break
        ret.add_result(valid == False, name='init_simulation should remove all particles')

        for i in range(10):
            x = random.uniform(2.0, 1000.0)
            y = random.uniform(2.0, 1000.0)
            pids.append(core.add_particle(x, y))
            pids.append(core.add_particle(2.0, y))
            pids.append(core.add_particle(x, 2.0))
            pids.append(core.add_particle(-2.0, y))
            pids.append(core.add_particle(x, -2.0))
        core.simulate_one_step()
        valid = False
        for pid in pids:
            p, valid = core.query_particle(pid)
            if valid != False:
                break
        ret.add_result(valid == False, name=f'simulate_one_step should remove all distant particles. Particle {pid} valid {valid} pos {p.pos}')

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

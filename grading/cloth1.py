import inspect
import numpy as np
import math
import random
from base import *
import util
from scipy.spatial.transform import Rotation
from progressbar import progressbar

def create_core(module):
    c = module.ClothCore()
    c.init_simulation()
    return c, c.reference_sim_parameters()

def _prepare(module):
    core, simp = create_core(module)
    return core, simp

def _ref_prepare():
    try:
        from pypsim_ref.cloth1 import ClothCore as RefClothCore
    except ModuleNotFoundError as e:
        print(e)
        print("This script should be executed under bin/. Example")
        print("psim/bin$ ../grading/psim_grade.py --case goo1 .....")
        exit()
    c = RefClothCore()
    c.init_simulation()
    return c, c.reference_sim_parameters()

def _set_single_constraint(simp, c_name):
    ALL_CNAMES = ['gravity', 'pin', 'stretch', 'bending', 'pulling']
    assert c_name in ALL_CNAMES
    setattr(simp, f'{c_name}Enabled', True)
    for cn in ALL_CNAMES:
        if cn == c_name:
            continue
        setattr(simp, f'{cn}Enabled', False)
        if cn == 'gravity':
            setattr(simp, f'{cn}G', 0.0)
        else:
            setattr(simp, f'{cn}Weight', 0.0)

class BiCore(object):
    def __init__(self, core, refcore):
        self.core = core
        self.refcore = refcore
        self.simp = self.core.reference_sim_parameters()
        self.refsimp = self.refcore.reference_sim_parameters()

    def set_simp(self, name, val):
        setattr(self.simp, name, val)
        setattr(self.refsimp, name, val)

    def set_single_constraint(self, c_name):
        _set_single_constraint(self.simp, c_name)
        _set_single_constraint(self.refsimp, c_name)

    def attach_mesh(self, V, F, scale):
        self.core.attach_mesh(V, F, scale)
        self.refcore.attach_mesh(V, F, scale)

    def simulate_one_step(self):
        self.core.simulate_one_step()
        self.refcore.simulate_one_step()

    def compare(self, rtol=1e-4):
        Q, _, _ = self.core.get_current_mesh()
        refQ, _, _ = self.refcore.get_current_mesh()
        compare_result = np.allclose(Q, refQ, rtol=rtol)
        return compare_result

    def reset(self):
        self.core.init_simulation()
        self.refcore.init_simulation()

def _prepare_bicore(module):
    core,_ = _prepare(module)
    refcore,_ = _ref_prepare()
    bicore = BiCore(core, refcore)
    return bicore

'''
4.1 Position Based Dynamics Framework

In practice, only test gravity.
'''
class PositionBasedDynamics(TestBase):
    PTS = 10
    def do_test(self, module, ret):
        from pypsim_ref.cloth1 import loadOBJ
        bicore = _prepare_bicore(module)
        bicore.set_single_constraint('gravity')
        V, F = loadOBJ('../assets/cloth/rect-coarse.obj')
        V *= 50.0
        bicore.attach_mesh(V, F, scale=1.0)
        for i in range(100):
            bicore.simulate_one_step()
            ret.add_result(bicore.compare())

'''
4.1 Pin Based Constraint

Should behave like optimization process
'''
class PinConstraints(TestBase):
    PTS = 10
    def do_test(self, module, ret):
        from pypsim_ref.cloth1 import loadOBJ
        core, simp = _prepare(module)
        _set_single_constraint(simp, 'pin')
        V, F = loadOBJ('../assets/cloth/rect-coarse.obj')
        V *= 50.0
        TL = np.array([1.0,1.0,0.0])
        TR = np.array([-1.0,1.0,0.0])
        TL_index = np.argmax(V.dot(TL))
        TR_index = np.argmax(V.dot(TR))
        # print(f"TL_index {TL_index} TR_index {TR_index}")
        # print(f"TL {V[TL_index]} TR {V[TR_index]}")
        core.attach_mesh(V, F, scale=1.0)
        rng = np.random.default_rng(seed=0xd7fbe4d90e3df2e0)
        for t in progressbar(range(100)):
            core.attach_mesh(V, F, scale=1.0)
            off = rng.uniform(low=-1.0,high=1.0, size=3)
            # off = rng.uniform(low=-1.0,high=1.0, size=3)
            core.update_mesh_vertics(V + off)
            Q, _, _ = core.get_current_mesh()
            loss = np.linalg.norm(Q[TL_index] - V[TL_index])
            loss += np.linalg.norm(Q[TR_index] - V[TR_index])
            # print(f"[INIT] loss: {loss} TL {Q[TL_index]} TR {Q[TR_index]}")
            for i in range(4):
                core.simulate_one_step()
                Q, _, _ = core.get_current_mesh()
                loss = np.linalg.norm(Q[TL_index] - V[TL_index])
                loss += np.linalg.norm(Q[TR_index] - V[TR_index])
                # print(f"[{i:05d}] loss: {loss} TL {Q[TL_index]} TR {Q[TR_index]}")
            compare_result = np.allclose(loss, 0.0)
            ret.add_result(compare_result)

def _surface_areas(V, F):
    edges_1 = V[F[:,1],:] - V[F[:,0],:]
    edges_2 = V[F[:,2],:] - V[F[:,0],:]
    return np.linalg.norm(np.cross(edges_1, edges_2), axis=1)

class StretchingConstraint(TestBase):
    PTS = 25
    def do_test(self, module, ret):
        from pypsim_ref.cloth1 import loadOBJ
        core, simp = _prepare(module)
        _set_single_constraint(simp, 'stretch')
        V, F = loadOBJ('../assets/cloth/rect-coarse.obj')
        nfaces = F.shape[0]
        V *= 50.0
        init_surface_areas = _surface_areas(V, F)
        print()
        print(init_surface_areas)
        rng = np.random.default_rng(seed=0xd7fbe4d90e3df2e0)
        rng_faces = rng.integers(nfaces, size=100)
        for t in progressbar(range(10)):
            core.attach_mesh(V, F, scale=1.0)
            off = rng.uniform(low=-10.0,high=10.0, size=3)
            rot = Rotation.random(random_state=rng)
            Q = rot.apply(V) + off
            Q = np.copy(V)
            fi = F[rng_faces[t],:]
            while True:
                bary = rng.uniform(low=0.0,high=1.0, size=3)
                norm = np.linalg.norm(bary)
                if norm > 1e-3:
                    break
            bary /= norm
            verts = np.array([Q[fi[0]], Q[fi[1]], Q[fi[2]]])
            # print(f"fi[0]: {fi[0].shape}")
            # print(f"Q[fi[0]]: {Q[fi[0]].shape}")
            # print(f"verts: {verts.shape}")
            # print(f"bary: {bary.shape}")
            v = bary.dot(verts)
            extend = np.exp(rng.uniform(low=np.log(0.01), high=np.log(10.0), size=3))
            diag = np.diag(extend)
            # diag = np.eye(3)
            # print(diag.shape)
            # Q = (Q - v).dot(diag) + v
            # diag = np.diag([2.0, 1.0, 1.0])
            Q = (Q - v).dot(diag) + v
            core.update_mesh_vertics(Q)
            for i in range(3000):
                core.simulate_one_step()
                Q, _, _ = core.get_current_mesh()
                streching_surface_areas = _surface_areas(Q, F)
                loss = np.sum(streching_surface_areas-init_surface_areas) / nfaces
                # print(f"[{i:05d}] areas: {streching_surface_areas}")
                # print(f"[{i:05d}] loss: {loss}")
                compare_result = np.allclose(loss, 0.0, atol=1e-2)
                if compare_result:
                    break
            print(f"compare_result: {compare_result}, final loss: {loss}")
            ret.add_result(compare_result)

class BendingConstraints(TestBase):
    PTS = 35
    def do_test(self, module, ret):
        from pypsim_ref.cloth1 import loadOBJ, saveOBJ, principal_curvature
        core, simp = _prepare(module)
        _set_single_constraint(simp, 'bending')
        # simp.bendingWeight = 1.0
        V, F = loadOBJ('../assets/cloth/rect-coarse.obj')
        nfaces = F.shape[0]
        nverts = F.shape[0]
        V *= 50.0
        rng = np.random.default_rng(seed=0xd7fbe4d90e3df2e0)
        def get_loss(Q):
            _,_,PV1, PV2, bad = principal_curvature(Q, F)
            # Mean principle curvature, doesn't work very well
            # return np.sum(np.abs(PV1) + np.abs(PV2)) / nfaces
            # Maximum principle curvature, also doesn't work very well
            ret = max(np.max(np.abs(PV1)), np.max(np.abs(PV2)))
            return ret
            _, ret, _, _ = np.linalg.lstsq(Q[:, :2], Q[:,2], rcond=None)
            return ret
        for t in range(10):
            core.attach_mesh(V, F, scale=1.0)
            found = False
            while not found:
                fi = rng.integers(nfaces, endpoint=False)
                vi = F[fi]
                bary = rng.uniform(low=0.0,high=1.0, size=3)
                norm = np.linalg.norm(bary)
                if norm <= 1e-3:
                    continue
                bary /= norm
                verts = np.array([V[vi[0]], V[vi[1]], V[vi[2]]])
                v = bary.dot(verts)
                for i in range(4):
                    cut_angle = rng.uniform(math.pi / 8, math.pi / 2)
                    cut_axis = np.array([math.cos(cut_angle), math.sin(cut_angle), 0.0])
                    side = np.cross(V - v, cut_axis).dot(np.array([0,0,1])) > 0
                    size_int = side.astype(int)
                    nsep = np.sum(size_int)
                    print(f"nsep {nsep} nverts {nverts}")
                    if nsep < nverts * 0.05 or nsep > nverts * 0.95:
                        '''
                        reject tiny cuts.
                        '''
                        continue
                    bend_angle = rng.uniform(math.pi / 8, math.pi / 2)
                    bend_angle *= rng.choice([-1.0, 1.0])
                    bend_array = (bend_angle * size_int).reshape((1, size_int.shape[0]))
                    print(f"verts: {verts}")
                    print(f"v : {v}")
                    print(f"bend_angle : { bend_angle}")
                    print(f"bend_array : { bend_array.shape}")
                    rotvec_array = cut_axis.reshape((3,1)).dot(bend_array).transpose()
                    rots = Rotation.from_rotvec(rotvec_array)
                    Q = rots.apply(V - v) + v
                    if get_loss(Q) < 0.1:
                        continue
                    found = True
                    break
            init_loss = get_loss(Q)
            print(f"Init loss {init_loss}")
            # continue
            # Q = rots.apply(V - v)
            # z_delta = np.zeros(shape=(size_int.shape[0], 3))
            # z_delta[:, 2] = size_int
            # z_delta[:, 2] = bend_array[0,:]
            # Q = (V - v) # + z_delta
            # saveOBJ(Q, F, '0.obj')
            # exit()

            core.update_mesh_vertics(Q)
            for i in range(30000):
                core.simulate_one_step()
                # print(f"[{i:05d}] areas: {streching_surface_areas}")
                if (i + 1) % 100 == 0:
                    Q, _, _ = core.get_current_mesh()
                    # _,_,PV1, PV2, bad = principal_curvature(Q, F)
                    # Mean principle curvature, doesn't work very well
                    # loss = np.sum(np.abs(PV1) + np.abs(PV2)) / nfaces
                    # Maximum principle curvature, also doesn't work very well
                    # loss = max(np.max(np.abs(PV1)), np.max(np.abs(PV2)))
                    # _, loss, _, _ = np.linalg.lstsq(Q[:, :2], Q[:,2], rcond=None)
                    loss = get_loss(Q)
                    # compare_result = np.allclose(loss, init_loss * 0.10, atol=1e-2)
                    compare_result = (loss <= init_loss * 0.05)
                    print(f"[{(i+1):05d}] compare_result: {compare_result}, final loss: {loss}, bendingWeight {simp.bendingWeight}")
                    # saveOBJ(Q, F, f'{(i+1)//100}.obj')
                    # simp.bendingWeight *= 0.99
                    if compare_result:
                        break
            ret.add_result(compare_result)

class DragConstraints(TestBase):
    PTS = 20
    def do_test(self, module, ret):
        from pypsim_ref.cloth1 import loadOBJ
        core, simp = _prepare(module)
        _set_single_constraint(simp, 'pulling')
        V, F = loadOBJ('../assets/cloth/rect-coarse.obj')
        nverts = V.shape[0]
        V *= 50.0
        rng = np.random.default_rng(seed=0xf1bf30cc0f25ae6d)
        for trial in range(100):
            core.attach_mesh(V, F, scale=1.0)
            vert_index = rng.integers(low=0, high=nverts, endpoint=True)
            '''
            Suppose we need a curve f_{x,y,z}(t).
            We start with generating a 3x3 matrix [a b c], where
            f_x(t) = a_x t^3 + b_x t^2 + c_x t + x0
            f_y(t) = a_y t^3 + b_y t^2 + c_y t + y0
            f_z(t) = a_z t^3 + b_z t^2 + c_z t + z0
            '''
            curve_matrix = rng.uniform(low=-10, high=10, size=(3,3))
            '''
            '''
            M = np.empty(shape=(3,4))
            M[:3, :3] = curve_matrix
            M[:, 3] = V[vert_index, :]
            speed = np.exp(rng.uniform(low=np.log(0.01), high=np.log(10.0)))
            core.hold(vert_index, V[vert_index, :]);
            for i in range(0, 100):
                t = speed * float(i) / 100.0
                coe = np.array([t ** 3, t ** 2, t, 1])
                p = M.dot(coe)
                core.update_hold(p)
                for k in range(3):
                    core.simulate_one_step()
                Q, _, _ = core.get_current_mesh()
                val = Q[vert_index]
                exp = p
                compare_result = np.allclose(val, exp, rtol=1e-2)
                print(f"compare_result {compare_result} val {val} expect {exp}")
                ret.add_result(compare_result)

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
        # print(f"{name}")
        if not inspect.isclass(obj):
            continue
        if not issubclass(obj, TestBase) or obj == TestBase:
            continue
        test_objects.append(obj())
    total_pts = np.sum([t.report_total_points() for t in test_objects])
    # assert total_pts == 100, f'Total points {total_pts} != 100'
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

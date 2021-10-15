from RKJ_utils import *
from yade import *
from yade import plot
from math import *

import json
import sys, os

sys.path.append(".")
          
# ---------------------------------------------------------------------------- Simulation mode
simulation_mode = 'batch' # {'single', 'batch'}
diagnose        = False
stopSimulation  = True

if simulation_mode == 'single':
    ID     = 1
    young  = 2.7e6
    Wedge_name     = 60
    brush_friction = 44
    print_frction  = 24
    radius_diff    = 0.1e-3
    Max_depth      = 33e-3


elif simulation_mode == 'batch':
    readParamsFromTable(
        ID     = 1,
        young  = 2.7e6,   
        Wedge_name     = 60,
        brush_friction = 44,
        print_frction  = 24,
        radius_diff    = 0.1e-3,
        Max_depth      = 33e-3
    )
    from yade.params.table import *

# The following are used for diagnosis
time_at_record = 0
time_recorded  = 0
time_at_record_stop = 0


# ---------------------------------------------------------------------------- input parameter
# ----------------------------------------------------- brush
bristle_radius_root    = 1e-3   + radius_diff
bristle_radius_diff    = 0.5e-3 - radius_diff
bristle_radius_tip     = bristle_radius_root - bristle_radius_diff
bristle_length         = 35e-3
bristle_no_of_segments = 12

bristle_young    = young
bristle_density  = 1700
bristle_poisson  = 0.48
bristle_friction = radians(brush_friction)
bristle_damping  = 0

bristles_dx      = 2.4375e-3
bristles_dy      = 2.4375e-3
bristle_x_no     = 17
bristle_y_no     = 5

brush_height = 12.0e-3
covar        = 0e-7
clearance    = 1e-5

brush_position = [0.0, 0.0, 0.0]
orientation    = Quaternion((1,0,0), 0)
# ----------------------------------------------------- target
target_young    = 0.001e9
target_density  = 1000
target_poisson  = 0.3
target_friction = radians(print_frction)

p_radius     = 0.2e-3
target_shape = 'wedge_' + str(Wedge_name) + '.gts'
geometry_location = './geometry/'
target_shape      = geometry_location + target_shape 

# ----------------------------------------------------- Experiment 
maxPenetrationDepth = Max_depth
penetrationStepSize = 1e-3
velocity_direction  = -1
maxPenetrationVelocity  = 0.02
PenetrationAcceleration = 0.5e-1 # acc = 6 * x / t**2 for t = 1, x = 1e-3

relaxationTime  = 1e-4

# We consider the force are steady when its consecutive readings have a standard deviation
# less than a specified quantity. This SD and the number of consecutive readings are
# defined below  
maxSDtoConsiderSS   = 1e-4
noOfPointsToCheckSD = 50

jitter_wavelength = 1e-3
jitter_amp        = 0
wave_number       = (2 * pi)/jitter_wavelength

# ---------------------------------------------------------------------------- Materials
brush_int_mat  = 'cyl_int_mat'
brush_ext_mat  = 'cyl_ext_mat'

target_int_mat = 'pfacet_int_mat'
target_ext_mat = 'pfacet_ext_mat'

add_official_brush_material(
        young   = bristle_young, 
        poisson = bristle_poisson, 
        density = bristle_density, 
        friction_Angle   = bristle_friction,
        ext_mat_name   = brush_ext_mat, 
        int_mat_name   = brush_int_mat,
    )


add_pfacet_material(
        young   = target_young, 
        poisson = target_poisson, 
        density = target_density, 
        friction_Angle = target_friction, 
        int_mat_name   = target_int_mat, 
        ext_mat_name   = target_ext_mat
    )
# ---------------------------------------------------------------------------- User defined functions
# ----------------------------------------------------- isSteady
class isSteady:
    '''
    Records varying data and checks when this data has reached steady state
    '''
    # ------------------------------------------- init
    def __init__(self, array_size, maxSD):
        self.array_size = array_size
        self.maxSD = maxSD
        self.initialise()

    # ------------------------------------------- initialise  
    def initialise(self):
        self.data = list(np.random.normal(0.0, 10 * self.maxSD, self.array_size))

    # ------------------------------------------- isStable  
    def isStable(self, datum):
        '''
        Adds a datum to the data and returns if the data is stable i.e. if its SD
        is less than acceptable maxSD
        '''
        self._add(datum)
        sd = np.std(self.data)
        # print(sd)

        if sd > self.maxSD:
            return False
        else:
            self.initialise()
            return True

    # ------------------------------------------- _add
    def _add(self, datum):
        '''
        Shifts the data array and add the datum to its end while removing the first member
        '''
        self._shiftFront()
        self.data[-1] = datum

    # ------------------------------------------- _shiftFront
    def _shiftFront(self):
        '''
        Cyclic shift. The elements of array are shifted back i.e. ith member becomes
        (i-1)th member
        '''
        temp = self.data[:]
        for i in range(self.array_size):
            self.data[self.array_size - i - 2] = temp[self.array_size - i - 1]

# ----------------------------------------------------- dispatcher
def dispatcher():
    '''
    Oversees the experiment. Transfers control to function to move and then 
    Transfers control to function to record. Add the following to the engines list

        PyRunner(
        command    = 'dispatcher()',
        virtPeriod = O.dt, 
        label      = 'experimentEngine')
        
    '''
    global motionStartTime, nextStep, PenetrationAcceleration, velocity_direction, stopSimulation, time_at_record, time_at_record_stop, time_recorded

    if nextStep == 'move':
        motionStartTime = O.time
        nextStep        = 'record'
        experimentEngine.command = 'mover()' # transferring control to mover()
        time_at_record_stop      = O.time 
        time_recorded           += time_at_record_stop - time_at_record

    elif nextStep == 'record':
        pos   = O.bodies[target_bottom_id].state.pos - target.initPos

        if abs(abs(pos[2]) - maxPenetrationDepth) < penetrationStepSize/20:
            velocity_direction   = 1         

        nextStep                 = 'move'
        time_at_record           = O.time
        experimentEngine.command = 'recorder()'  # transferring control to recorder()

    if stopSimulation:
        pos   = O.bodies[target_bottom_id].state.pos - target.initPos
        if pos[2] > penetrationStepSize:
            stopper()

# ----------------------------------------------------- mover
def mover():    
    '''
    Sets the motion of the target. When the target stops, it sets the global variable 
    state to stop and relinquishes control to dispatcher
    '''
    # calculating the jigger velocity
    pos        = O.bodies[target_bottom_id].state.pos - target.initPos
    jitter_vel = calcJitterVel(pos)

    # calculating the velocity magnitude using the calcvel function
    time                            = O.time - motionStartTime
    speed, next_virt_perdiod, state = calcVelParabolic(time)
    for id in target.mover_list:
        O.bodies[id].state.vel      = speed * Vector3([0, jitter_vel, velocity_direction])

    experimentEngine.virtPeriod     = next_virt_perdiod
        
    if state == 'stop':
        experimentEngine.command    = 'dispatcher()'  # transferring control to dispatcher()

# ----------------------------------------------------- calcJitterVel
def calcJitterVel(pos):
    '''
    Calculates the sideways jitter velocity based in the position of the body
    '''
    # Jitter_amp is the amplitude of the jitter position not the velocity
    return jitter_amp * wave_number * cos(wave_number * pos[2])
  
# ----------------------------------------------------- calcVelParabolic
def calcVelParabolic(time):
    '''
    Takes in the time and returns the velocity based on a parabolic profile
    '''
    x  = float(penetrationStepSize)
    am = float(PenetrationAcceleration)
    tn = (6.0 * x / am)**0.5
    k  = am/tn

    if time <  tn:
        currentVelocity   = -k * (time * (time - tn))
        next_virt_perdiod = O.dt
        state = 'move'

    else:
        currentVelocity   = 0.0
        next_virt_perdiod = relaxationTime
        state = 'stop'

    return currentVelocity, next_virt_perdiod, state

# ----------------------------------------------------- recorder
def recorder():
    '''
    Calls function to graph/record data when steady state is reached. Once recorded, it 
    relinquishes control to the dispatcher
    '''
    canRecord = steadyStateQ()
    if canRecord:
        graph()
        experimentEngine.virtPeriod = O.dt
        experimentEngine.command    = 'dispatcher()' # transferring control to dispatcher()

# ----------------------------------------------------- stopper
def stopper():
    result_dict               = {}
    result_dict['parameters'] = getParameters(globals())
    result_dict['data']       = plot.data
    
    file_json = './res/Sim_result_ID_' + str(ID) + '.json'
    with open(file_json, 'w') as f:
        json.dump(result_dict, f, indent=4, cls=VectorEncoder)

    O.pause()

# ---------------------------------------------------------------------------- Engines
O.engines = [
                ForceResetter(),

                InsertionSortCollider([
                    Bo1_GridConnection_Aabb(),
                    Bo1_PFacet_Aabb(),
                    Bo1_Sphere_Aabb(),
                ]),

                InteractionLoop(
                    [
                        Ig2_PFacet_PFacet_ScGeom(),
                        Ig2_GridConnection_GridConnection_GridCoGridCoGeom(),
                        Ig2_GridNode_GridNode_GridNodeGeom6D(),
                        Ig2_GridConnection_PFacet_ScGeom(),
                        Ig2_Sphere_PFacet_ScGridCoGeom(),
                    ],
                    [
                        Ip2_FrictMat_FrictMat_FrictPhys(),
                        Ip2_CohFrictMat_CohFrictMat_CohFrictPhys(
                            setCohesionNow = True, 
                            setCohesionOnNewContacts = False
                            ),
                    ],
                    [
                        Law2_GridCoGridCoGeom_FrictPhys_CundallStrack(),
                        Law2_ScGeom_FrictPhys_CundallStrack(),
                        Law2_ScGridCoGeom_FrictPhys_CundallStrack(),
                        Law2_ScGeom6D_CohFrictPhys_CohesionMoment(),
                    ],
                ),                
] 


# ---------------------------------------------------------------------------- objects 
# ----------------------------------------------------- target
target = data()
(
    target.nodeIds,
    target.cylIds,
    target.pfIds,
) = create_GTSPfacet(
    target_shape,
    radius = p_radius,
    shift  = (0,    bristles_dy/2,  bristle_length + bristle_radius_tip + p_radius),
    scale  = 1,
    wire   = False,
    fixed  = False,
    color  = [0.1,0.5,0.1],
    materialNodes = target_int_mat,
    material      = target_ext_mat,
)

target.ids = target.nodeIds + target.cylIds + target.pfIds
target.clump_id = O.bodies.clump(target.ids)

for i in target.ids + [target.clump_id]:
    O.bodies[i].dynamic = False

target_bottom_id  = target.clump_id # 0
target.initPos    = O.bodies[target_bottom_id].state.pos
target.mover_list = [target.clump_id]
# ----------------------------------------------------- brush
brush_x_length     = bristles_dx * bristle_x_no
brush_y_length     = bristles_dy * bristle_y_no

brush_x_no_density = 1 / bristles_dx
brush_y_no_density = 1 / bristles_dy
bristle_tip_spread_covariance = [[covar, 0.0],[0.0, covar]]

brush = ccp_brush(
        bristle_radius   = bristle_radius_tip,
        bristle_radius_2 = bristle_radius_root, 
        bristle_length   = bristle_length,
        no_of_segment    = bristle_no_of_segments, 

        bristle_external_material = brush_ext_mat,
        bristle_internal_material = brush_int_mat,        

        x_lenght = brush_x_length, 
        y_length = brush_y_length, 
        x_no_density = brush_x_no_density, 
        y_no_density = brush_y_no_density, 

        brush_base_pos = brush_position,
        clearance      = clearance,        
        orientation    = orientation,

        root_fixed_Q = True, # If true the roots will be fixed to the ground
        bristle_tip_spread_covariance = bristle_tip_spread_covariance,# 'default',
    )
node_ids_array, cyl_ids_array, root_node_ids_array = brush.generate()

# ----------------------------------------------------------------------------- Additional engines
nextStep         = 'move'
motionStartTime  = 0.0

O.engines += [
    NewtonIntegrator(gravity = [0,0,0], damping = 0.4),   
    PyRunner(
        command    = 'dispatcher()',
        virtPeriod = O.dt, 
        label      = 'experimentEngine'),
     
]
# ----------------------------------------------------- graphing 
from yade import plot
prev_pos   = 0.0
prev_force = 0.0

if diagnose:
   plot.plots = {
    'pos_z':('force_z'), 
    'time_during_relaxation':('Force_during_relaxation_new_0.1')
    }

else:
    plot.plots = {
        'pos_z':('force_z')
    }

      
def graph():
    force = Vector3(0,0,0)
    pos   = O.bodies[target_bottom_id].state.pos - target.initPos
    for id in root_node_ids_array:
        force += O.forces.f(id)

    if diagnose:
        # ---- Diagnosis
        global prev_force, prev_pos
        plot.addData(pos_z = prev_pos, force_z = prev_force)    
        prev_pos   = pos[2]
        prev_force = force[2]
        # ---- Diagnosis

    plot.addData(pos_z = pos[2], force_z = force[2]) 

graph()
plot.plot()

# ----------------------------------------------------- steadyStateQ
canMove = isSteady(noOfPointsToCheckSD, maxSDtoConsiderSS)
def steadyStateQ():
    '''
    Checks if steady state is reached
    '''
    global canMove
    force = Vector3(0,0,0)

    for id in root_node_ids_array:
        force += O.forces.f(id)

    if diagnose:
        # ---- Diagnosis
        time  = O.time
        plot.addData(time_during_relaxation = time - time_at_record + time_recorded, Force_during_relaxation = force[2])
        # ---- Diagnosis

    return canMove.isStable(force[2])

graph()
plot.plot()
# ----------------------------------------------------------------------------- GUI controller 
if simulation_mode == 'single':
    from yade import qt
    qt.Controller()
    qtv          = qt.View()
    qtr          = qt.Renderer()
    qtr.light2   = True
    qtr.lightPos = Vector3(1200,1500,500)
    qtv.ortho    = True # orthographic view in viewport
    Gl1_Sphere.stripes = True

# ----------------------------------------------------------------------------- simulation controls
O.dt = 2.0e-5 # utils.PWaveTimeStep()

if simulation_mode == 'batch':
    O.run()
    waitIfBatch()

else:
    O.saveTmp()


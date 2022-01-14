# import library yang dibutuhkan
import numpy as np
import pandas as pd
from tqdm import tqdm
from matplotlib import pyplot as plt
from scipy.signal import find_peaks

# initial value dalam fungsi PID
def PID(Kp=1, Ti=1, Td=1, t_initial=0, u_bar=0, control_type='pid'):
    """Basic PID controller
    args:
    - Kp (integer atau float) default: 1 = proportional gain
    - Ti (integer atau float) default: 1 = integral period
    - Td (integer atau float) default: 1 = derivative period
    - t_initial (integer atau float) default: 0 = initial time
    - u_bar (integer atau float) default: 0 = base line for control signal
    - control_type (string) default: pid = choose control type, either 'p', 'pi', 'pd', or 'pid'

    yield:
    - u (float) = control signal

    generator send():
    - t, feedback, ref (list-like type) = time, feedback (output from model), and reference value
    """
    if not(control_type == 'p' or control_type == 'pi' or control_type == 'pd' or control_type == 'pid'):
        raise ValueError("'control_type' arg only accepts either 'p','pi','pd','pid'")

    # initialize stored data
    e_prev = 0
    t_prev = t_initial #-100
    I = 0

    # initial control
    u = u_bar

    while True:
        # yield MV wait for new t, PV, SP
        t, feedback, ref = yield u

        # PID calculation
        e = ref - feedback
        de = e - e_prev
        dt = t - t_prev

        P = Kp * e
        I = I + e*dt
        D = de / dt

        if control_type == 'p':
            u = u_bar + P
        elif control_type == 'pi':
            u = u_bar + Kp*(e + (1/Ti)*I)
        elif control_type == 'pd':
            u = u_bar + Kp*(e + Td*D)
        else:
            u = u_bar + Kp*(e + (1/Ti)*I + Td*D)

        # store updated value for next iteration
        e_prev = e
        t_prev = t

def car_dynamics(b, v_in, u, m, dt):
    """Vehicle dynamics model

    args:
    - b (integer atau float) = konstanta gesek (Nm/s)
    - v_in (integer atau float) = input/initial speed (m/s)
    - u (integer atau float) = sinyal kontrol (N)
    - m (integer atau float) = massa kendaraan (kg)
    - dt (integer atau float) = difference of time (s)

    return:
    v_out (float) = output / final speed (m/s)
    """
    temp = (-b*v_in + u)/m
    v_out = temp*dt + v_in
    return v_out

# define constants - nilai input
Kp = 1950
#Ti = 0.5
#Td = 0.1
mass = 1000 #kg
frict = 50 #Nm/s

# declare PID controller
controller = PID(Kp=Kp, Ti=Ti, Td=Td)
controller.send(None) # initialize controller

# set speed reference
ref = np.ones(shape=50) * 5

# define useful array variables
feedback = np.zeros(shape=ref.shape)
times = np.arange(start=1, stop=len(ref), step=1)
u = np.zeros(shape=ref.shape)
error = np.zeros(shape=ref.shape)

t_prev = 0 # buffer to store previous time

# loop through each time t
for t in tqdm(times):
    error[t] = ref[t-1] - feedback[t-1] # get error at time t
    u[t] = controller.send([t, feedback[t-1], ref[t-1]]) # PID calculation
    feedback[t] = car_dynamics(b=frict, v_in=feedback[t-1], u=u[t], m=mass, dt=t-t_prev)
    t_prev = t # store for next iteration


# grafik reference vs feedback

#plt.figure(figsize=(10,7))
#line1, = plt.plot([0] + times.tolist(), ref)
#line2, = plt.plot([0] + times.tolist(), feedback)
#plt.legend([line1, line2], ['Reference', 'Feedback'])
#plt.grid()
#plt.title("Reference vs Model Feedback")
#plt.xlabel("Time (s)")
#plt.ylabel("Speed (m/s)")

# grafik fungsi u

#plt.figure(figsize=(10,7))
#plt.plot([0] + times.tolist(), u)
#plt.legend()
#plt.grid()
#plt.title("Output of the controller (u)")
#plt.xlabel("Time (s)")
#plt.ylabel("Force (N)")

# grafik error

#plt.figure(figsize=(10,7))
#plt.plot([0] + times.tolist(), error)
#plt.legend()
#plt.grid()
#plt.title("Error between reference and model feedback")
#plt.xlabel("Time (s)")
#plt.ylabel("Error of speed (m/s)")


"""STEP 2: Get Ku & Tu and calculate Kp, Ti, Td"""
# find peaks in feedback signal
peaks, _ = find_peaks(feedback) # get idx of peaks
print('peaks: ', feedback[peaks])

# get Ku and Tu
Tu = np.mean(np.diff(peaks))
Ku = Kp
print('Ku = ', Ku)
print('Tu = ', Tu)
print()

# calculate Kp, Ti, Td using different control type
pid_control_type = {'p': [0.5*Ku, 0, 0],
'pi': [0.45*Ku, 0.8*Tu],
'pd': [0.8*Ku, 0, 0.125*Tu],
'classic_pid': [0.6*Ku, 0.5*Tu, 0.125*Tu],
'pessen_integral': [0.7*Ku, 0.4*Tu, 0.15*Tu],
'some_overshoot': [0.33*Ku, 0.5*Tu, 0.33*Tu],
'no_overshoot': [0.2*Ku, 0.5*Tu, 0.33*Tu]}

for key, value in pid_control_type.items():
    print(f"Control type '{key}':")
    print(f"Kp = {value[0] if value[0] != 0 else '_'}")
    print(f"Ti = {value[1] if value[1] != 0 else '_'}")
    print(f"Td = {value[2] if value[2] != 0 else '_'}")
    print('\n')

"""STEP 3: RUN ALL CONTROLLER TYPES USING CONSTANTS JUST OBTAINED"""
# buffer to store value
ref_list = []
feedback_list = []

# loop every controller type
for key, value in pid_control_type.items():
    # get constants
    Kp = value[0]
    Ti = value[1]
    Td = value[2]

    # set control type arg for PID function
    if key == 'p' or key == 'pi' or key == 'pd':
        control_type = key
    else:
        control_type = 'pid'

    print('==================================')
    print(f"Control type: {key}")
    print('==================================')

    # declare PID controller function
    controller = PID(Kp=Kp, Ti=Ti, Td=Td, control_type=control_type)
    controller.send(None) # Initialize controller

    # set speed reference
    ref = np.concatenate((np.ones(shape=50) * 2, np.ones(shape=50) * 10))

    # define useful array variables
    feedback = np.zeros(shape=ref.shape)
    times = np.arange(start=1, stop=len(ref), step=1)
    u = np.zeros(shape=ref.shape)
    error = np.zeros(shape=ref.shape)

    t_prev = 0 # buffer for time

    # another loop through each time t
    for t in times:
        error[t] = ref[t-1] - feedback[t-1] # get error at time t
        u[t] = controller.send([t, feedback[t-1], ref[t-1]]) # PID calculation
        feedback[t] = car_dynamics(b=frict, v_in=feedback[t-1], u=u[t], m=mass, dt=t-t_prev) # model output calculation
        t_prev = t # store for next loop

    # store ref and feedback
    ref_list.append(ref)
    feedback_list.append(feedback)

# graph for all 7 controller
fig, axs = plt.subplots(3, 3, figsize=(12,8))
axs[0,0].plot([0] + times.tolist(), ref_list[0])
axs[0,0].plot([0] + times.tolist(), feedback_list[0])
axs[0,0].set_title(list(pid_control_type.keys())[0])
axs[0,0].grid()

axs[0,1].plot([0] + times.tolist(), ref_list[1])
axs[0,1].plot([0] + times.tolist(), feedback_list[1])
axs[0,1].set_title(list(pid_control_type.keys())[1])
axs[0,1].grid()

axs[0,2].plot([0] + times.tolist(), ref_list[2])
axs[0,2].plot([0] + times.tolist(), feedback_list[2])
axs[0,2].set_title(list(pid_control_type.keys())[2])
axs[0,2].grid()

axs[1,0].plot([0] + times.tolist(), ref_list[3])
axs[1,0].plot([0] + times.tolist(), feedback_list[3])
axs[1,0].set_title(list(pid_control_type.keys())[3])
axs[1,0].grid()

axs[1,1].plot([0] + times.tolist(), ref_list[4])
axs[1,1].plot([0] + times.tolist(), feedback_list[4])
axs[1,1].set_title(list(pid_control_type.keys())[4])
axs[1,1].grid()

axs[1,2].plot([0] + times.tolist(), ref_list[5])
axs[1,2].plot([0] + times.tolist(), feedback_list[5])
axs[1,2].set_title(list(pid_control_type.keys())[5])
axs[1,2].grid()

line1, = axs[2,0].plot([0] + times.tolist(), ref_list[6])
line2, = axs[2,0].plot([0] + times.tolist(), feedback_list[6])
axs[2,0].set_title(list(pid_control_type.keys())[6])
axs[2,0].grid()

fig.delaxes(axs[2,1])
fig.delaxes(axs[2,2])

fig.legend((line1, line2), ('reference', 'feedback'), loc='lower right')

fig.tight_layout()
plt.show()
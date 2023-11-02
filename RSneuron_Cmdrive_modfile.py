from neuron import h, gui
from cellmodels.RSPY import sPY
import matplotlib.pyplot as plt
import numpy as np

# Creating cell
cell = sPY()

# Setting up US stimulation
us_start = 10
us_durn = 40
us_freq = 690 # Frequency in kHz. 
us_amp = 0.8 # amplitude of the sine wave for Cmdrive

# Transfer the FUSS details to the mod file parameters
cell.soma(0.5).dcdt.tbegin = us_start
cell.soma(0.5).dcdt.tdur = us_durn
cell.soma(0.5).dcdt.w = us_freq
cell.soma(0.5).dcdt.c2 = us_amp # This is the amplitude of the sine wave

# Time intervals
h.dt = 0.025 / us_freq

# Tstop defined based on above info
h.tstop = us_start + us_durn + 10

# Preparing the recoring variables
t = h.Vector().record( h._ref_t )
v = h.Vector().record( cell.soma(0.5)._ref_v )

# Initializing the variables
h.celsius = 36
h.finitialize( -71.8 )


# Running the simulation (also checking if the dt can be changed in between. 
h.continuerun(h.tstop)

# Now doing the window averaging to clean up the noise. 

Nw = us_freq # actual formula is 0.025 / h.dt; which will come to us_freq anyway. 
Nus = len( v )

resample_flag = 0
if (Nus-1) % Nw != 0:
    print( "length of vectors not a multiple of us_freq")
else:
    print( "Denoising (Window-averaging + Downsampling)..." )
    resample_flag = 1
    win_steps = Nus // Nw 
    t_new = [0]
    v_new = [v[0]]

    for i in range (win_steps):
        t_new.append( (i+1)*0.025 )
        v_new.append( np.mean( [ v[j] for j in range ((i)*Nw, (i+1)*Nw) ] ) )

# Plotting the results
if resample_flag:
    plt.figure()
    plt.plot( t_new, v_new, linewidth = 2, label = "Resampled Vm" )
    plt.xlabel("Time (ms)")
    plt.ylabel( "Vm (mV)" )
    plt.legend()
    plt.savefig("output_figs/response_denoised_vm.png")


plt.figure()

plt.figure()
plt.plot(t, v, linewidth = 2, label = 'Vm')
plt.xlabel("Time (ms)")
plt.ylabel("Potential (mV)")
plt.legend()
plt.savefig("output_figs/response_vm.png")

plt.show()



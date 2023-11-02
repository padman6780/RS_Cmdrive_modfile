from neuron import h

class sPY:
    """
    /*--------------------------------------------------------------
            TEMPLATE FILE FOR REGULAR-SPIKING CORTICAL PYRAMIDAL CELL
            ---------------------------------------------------------

            One compartment model and currents derived from:

       Pospischil, M., Toledo-Rodriguez, M., Monier, C., Piwkowska, Z.,
       Bal, T., Fregnac, Y., Markram, H. and Destexhe, A.
       Minimal Hodgkin-Huxley type models for different classes of
       cortical and thalamic neurons.
       Biological Cybernetics 99: 427-441, 2008.

            This model was inspired from:

       McCormick, D.A., Wang, Z. and Huguenard, J. Neurotransmitter
       control of neocortical neuronal activity and excitability.
       Cerebral Cortex 3: 387-398, 1993.

            - one compartment model
            - passive
            - HH: Traub
            - IM: m format

            Alain Destexhe, CNRS, 2008

    --------------------------------------------------------------*/

    """
    def __init__(self):
        self.soma = h.Section(name='soma')
        self.v_potassium = -100 # potassium reversal potential
        self.v_sodium = 50 # sodium reversal potential

        self.soma.Ra = 100 # geometry
        self.soma.nseg = 1
        self.soma.diam = 96
        self.soma.L = 96 # so that area is 43429.37684 um2
        self.soma.cm = 1
        h.celsius = 36 # This is the default temperature of pyramidal neurons. 

        self.soma.insert('pas') # leak current
        self.soma.e_pas = -70.3
        self.soma.g_pas = 2.05e-5 # Rin = 34 Meg

        # conversion with McC units:
        # g(S/cm2) = g(nS)*1e-9/29000e-8
        #          = g(nS) * 3.45e-6

        self.soma.insert('hh2') # Hodgin-Huxley INa and IK: Here an additional modification is done. Variable membrane capacitance is introduced to model ultrasonic neuromodulation.
        self.soma.ek = self.v_potassium
        self.soma.ena = self.v_sodium
        self.soma.vtraub_hh2 = -56.2 # Resting Vm, BJ was -55
        self.soma.gnabar_hh2 = 0.056 # McCormick=15 muS, thal was 0.09
        self.soma.gkbar_hh2 = 0.006 # spike duration of pyr cells

        self.soma.insert('im') # M current
        h.taumax_im = 608 # taumax_im is a GLOBAL variable
        self.soma.gkbar_im = 7.5e-5 # Diego's IM (copyrighted)

        print("\n<< sPY: passive, INa, IK, IM inserted >>\n")

        self.soma.insert('dcdt')
        self.soma(0.5).dcdt._ref_c = self.soma(0.5)._ref_cm
        
        print("\n<< sPY: dcdt mechanism inserted for Cm drive >>\n")


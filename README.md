# GN-ETSI-SimulationModel

Thesis report Link:  	http://alexandria.tue.nl/extra1/afstversl/wsk-i/Priandono_2015.pdf

The Intelligent Transport System goal is to make the transportation system more robust (e.g.
safe, secure, etc.). One of the methods allows the vehicles to communicate to each other. The
ETSI (Europe) and the IEEE (US) are two organizations that create the standard model for the
Inter-Vehicular Communication, which are the ETSI-ITS model and the WAVE model respect-
ively. Although the Multi-Channel Operation is supported, the ETSI standard document does
not contain any part that explains about the channel assignment mechanism to implement the
Multi-Channel Operation (MCO) on the ITS-G5 frequency.


This thesis contributes three things: 1) Build a VANET ETSI-ITS simulation model using
the NS3 library as a tool. 2) Define five strategies and compare them through simulation. 3)
Propose a new metric (i.e. accuracy) that can be used for a more practical approach. Moreover,
these five strategies are tested using two scenarios which are the single domain and the multi
domain scenario, and the results are measured using three metrics (i.e. reliability, accuracy, and
effectiveness).


The VANET ETSI-ITS simulation model simulates all the possible conditions in the ETSI-ITS
model. These possible conditions are the 802.11p broadcast back off procedure, the channel-load
information sharing, the ETSI-ITS cross layer architecture, propagation model, etc. The results
showed one strategy came as the best strategy because it shows the best performance in most
metrics. Meanwhile, two strategies present a unique behavior that can be used by a specific user.

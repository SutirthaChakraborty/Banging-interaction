# Banging Interaction
## _A ubimus-design strategy for the musical internet_

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

We explore the intersection of timbre-oriented design thinking with the harsh requirements of
casual interaction in transitional settings. We present the results of deploying a creative-action
metaphor based on whole-body interaction, the Dynamic Drum Collective [1]. An interesting feature of this new prototype is its ability to adapt to the particularities of
its usage. In a sense, the software is tuned by using it. Its adjustments are kept aligned with the
specific demands of the participant’s body characteristics and the spatial relationships between
the parts of the body. The Dynamic Drum Collective metaphor provides an example of a new
approach to design, Banging Interaction.


## Features

- Gesture based drumming
- Realtime interaction
- Multiplayer Drum Machine
- Works in CPU environment
- Code written in python

## Installation
To install create a conda environment. 
```sh
$ conda install -c conda-forge shapely
```
 
Install the other dependencies 
The following command will install the packages according to the configuration file requirements.txt.

```sh
$ pip install -r requirements.txt
```
## Execution
Ensure the server and the client code is running in the same network. We can check that using `ping`.
>  Sometimes antivirus doesn't allow, you might have to deactivate the firewall during the experiment.

#### Step 1: Run the Server
Check the server port 
> line 76 - 78
```sh
$ python3 ServerV2.py
```
#### Step 2: Run the Clients
Check the file drum.py. The `host` should be the IP address of the server machine.`port` should be same as port as mentioned in server. 
> line 87-89
```
host='192.168.0.241'
port = 1244
```
```sh
$ python3 drum.py
```

> [1] Chakraborty, S., Yaseen, A., Timoney, J., Lazzarini, V., & Keller, D. (2022). Adaptive touchless
whole-body interaction for casual ubiquitous musical activities. In Proceedings of the
International Computer Music Conference (ICMC2022), pp. 132–138. Limerick, Ireland:
University of Limerick
## Development

Want to contribute? Great! Create a branch and request pull.

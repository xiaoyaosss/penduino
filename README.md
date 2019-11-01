![alt text][logo]

# penduino

Microntroller-based pendulum experiment with magnetic drive and rotary encoder

## Installation

### Step 1 

This project should be installed in your arduino sketch area. It has been exported, so the supporting configuration may need changing to suit your particular arduino device.

### Step 2: Assemble the circuit

Assemble the circuit as follows (please check circuit before energising - this was drawn from memory):

![alt text][cct]

The circuit, winding and magnet details are modified (slightly) from [Tony Gasparovic's magnetic pendulum](http://nutsvolts.texterity.com/nutsvolts/200909/?folio=36&pg=36#pg36)

###Step 3: Load the code

Upload the code contained in this sketch on to your board, and fire away.

## Notes

0. Avoid over-voltage on the rotary encoder
0. The bump feature is omitted from the circuit diagram because it did not work as expected.

[status]: https://img.shields.io/badge/beta "Beta" 
[logo]: ./img/logo.png "penduino logo"
[cct]: ./img/cct.png "Circuit diagram"
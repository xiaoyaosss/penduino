![alt text][logo]

# penduino

Microntroller-based pendulum experiment with magnetic drive and rotary encoder

![alt text][swinging]


## Installation

### Step 1: Assemble the circuit & pendulum

Assemble the circuit using veroboard (there is no PCB design yet, but the Kicad schematic is part of this repository). However you assemble the board, please check for short circuits before powering up. Pay special attention to the polarity of transistors, LEDs and the diode.

![alt text][cct]

There are currently no plans available for the hardware, but the winding and magnet details are similar to [Tony Gasparovic's magnetic pendulum](http://nutsvolts.texterity.com/nutsvolts/200909/?folio=36&pg=36#pg36). The encoder is a generic OEM quadrature rotary encoder.


### Step 2: Obtain the code 


##### Offline editor:
Download the repository, then open the ```./sketch/sketch.ino``` in your offline arduino editor. Select your board type (e.g. Arduino Nano), and port.

##### Online editor:
Up all the files in the ```./sketch``` directory to your online arduino editor. 

### Step 3: Compile / load

Compile the code and download to your arduino.

Step 4: , connect up the circuit and pendulum 

### Step 4: Run it

Connect up your carefully-checked circuit, the arduino and your pendulum hardware. Mine looks like this:

![alt text][pendulum]

Then power it up to set it running.

You'll observe the following on startup:

The load LED comes on, until the pendulum has completely stopped moving. The rotary encoder is zero'd at this point, then swinging starts (so you know calibration is done).

### Step 5: Control it

Open the serial monitor in your editor (whether online or offline), and issue some commands.

You can start, stop, and recalibrate the pendulum. You can control over how high the pendulum swings, and how fast it stops.

## Commands


### Example

An example session with the pendulum might involve setting it swinging and stopping it again, using the following commands

```
    #example session
   {"cmd":"interval","param":200}
   {"cmd":"drive","param":30}
   {"cmd":"start","param":10}
```

pause to watch the swinging, then set the brake strength and start braking to bring the pendulum to standstill

```
   {"cmd":"brake","param":70}
   {"cmd":"stop","param":"brake"}
```


### Start

Energise the coil for <param> milliseconds, to get the coil moving from a cold start

```{"cmd":"start","param":50}```

After the initial bump, the drive circuit will be turned on in accordance with the setting of the drive threshold, which can be set with

```{"cmd":"drive","param":30}```


### Stop with active braking

This stops the coil by producing a repelling force when the coil is approaching the centre line

{"cmd":"stop","param":"brake"}

You can tune the strength of the braking by setting the threshold at which the braking starts. The permissable values are between 0 - 100, with 100 producing the strongest braking force, and 0 producing no braking force at all.

{"cmd":"brake","param":30}

Note that setting the brake param does not invoke the brake - you must still issue the stop command selecting the "brake" option.


## Stop with short-circuited coil

Short-circuiting the coil loads the pendulum, slowing it down. This can be used when the encoder is out of calibration such that braking produces undesirable behaviour

```{"cmd":"stop","param":"loaded"}```

### Stop without using any aids 

This command leaves the coil open circuited, so you can observe the coil's energy decay via the intrinsic loss mechanisms only

```{"cmd":"stop","param":"unloaded"}```


### (re)Calibrate

The arduino can miss steps if the swing speed is high, and the effect accumulates over time. You can recalibrate (re-zero) the position count.

```{"cmd":"calibrate"}```

### Encoder data frequency

Set the delay in milliseconds between reports of the encoder position

``` {"cmd":"interval","param":200}```

The minimum practical value for web applications is in the region of 20 - 50 milliseconds, because of the overhead of processing incoming websocket messages. If collecting data locally, you may find a smaller value is feasible.


## Operating details

### Effect of the energised coil

The electromagnet (or coil), when energised, repels the magnet affixed to the bottom of the pendulum. When the coil is not energised, it is open-circuited and has no effect on the pendulum. We can selectively energise and de-energise the coil in order to control the swing of the pendulum. Switching the coil on when the pendulum is receding from the centre point has the effect of repelling it and maintaining or increasing teh swing height. Switching the coil on when the pendulum is approaching the centre point has the opposite effect, acting as a brake that slows the pendulum. Outside of the time when we are driving or braking, we leave the coil open-circuited so it does not affect the pendulum. The timing is illustrated in schematic form:

![alt text][drivingbraking]

### Effect of the short-circuited coil
If we short-circuit the coil when it is de-energised, then the approaching or receding magnet causes a current to flow, which extracts a small amount of work from the pendulum. This slows the pendulum down, whether it is travelling toward or away from the coil. We can use a short-circuited coil to bring a freely swinging pendulum to rest, faster than if we left the coil open-circuited. It is not as fast as using the braking action, but the advantage of the short-circuited coil is that we can use it when the encoder is out of calibration, to bring it to rest for a recalibration. 

### Calibration

We assume that a free-swinging pendulum rests at the lowest potential energy position, i.e. hanging straight down. If we wait for the rotary encoder's output to stabilise on a fixed value, then we can assume we are at this point, and zero our position counter (that keeps track of the counts and their direction). If you have been swinging the pendulum very fast (e.g. looping the loop) then it is likely the arduino will miss encoder counts, and eventually the estimated position will be in such large error that the drive circuit will energise the coil at the wrong times to keep it swinging under control.

### Braking practical considerations

The braking routine includes an automatic soft-stop feature that detects a slow moving pendulum and gradually, and temporarily, reduces the braking threshold until the pendulum is centred. Without such an approach, the pendulum can occasionally end up bouncing back and forth across the edge of the braking threshold.

## Experiments to do with the pendulum

### Total energy

Estimate the height of the mass, and its velocity, from the encoder data and plot the kinetic, potential and total energy.

### Losses

The pendulum loses energy to air resistance and friction. You can estimate how much energy is lost on a single swing by comparing the peak swing heights with a de-energised coil. Does it depend on the swing height?

### Drive strength

What drive strength is required to compensate for the pendulum's losess? Infer the energy supplied by the coil by comparing swing heights and drive thresholds with the energy loss estimates you already generated.

### Consistency of period with amplitude

Pendulums only maintain a consistent oscillation period at small swing angles. What is the largest angle that you can swing at, and still get the same period of oscillation as a smaller amplitude swing?

### Chaos

Driven damped pendulums can exhibit chaotic behaviour, but can this driven damped pendulum do the same when the ability to apply a drive is restricted to a small subset of the travel range of the pendulum?

### Magnetic field strength

How closely does the coil follow the inverse third power law for magnetic field strength, and can that be inferred from analysing drive threshold versus swing height?
 
### Navigational uses

Accurate time keeping was required for navigation at sea using sightings of the the stars and sun. How well does the pendulum maintain the period, and phase, of its oscillations? Check the timing of the pendulum swings over a large time like an hour, or even 24 hours. Could you take your pendulum to see in a boat for use in navigation?

### Decay curves

When you have the pendulum swinging at a good amplitude, turn off the drive, and observe the decay in the swing amplitude. What pattern does it follow? What mathematical formula(e) could you use to describe it?

## Notes

0. Avoid over-voltage on the rotary encoder
0. On Linux, if you see access to the port denied due to lack of permissions, you may need to 
   - add your user to the group dialout
```
sudo usermod -aG dialout $USER
```
   - adjust the permissions after plugging in the arduino
```
sudo chmod a+rw <serial_port>
```
The serial port will be something like /dev/ttyUSB0


[status]: https://img.shields.io/badge/beta "Beta" 
[logo]: ./img/logo.png "penduino logo"
[cct]: ./img/cct.png "Circuit diagram"
[pendulum]: ./img/penduino.jpg "Pendulum in action"
[drivingbraking]: ./img/drivingbraking.png "Schematic timing diagram for driving coil"
[swinging]: ./img/swinging.gif "Pendulum swinging"
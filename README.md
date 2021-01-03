# MM2 Block-Control using Brake-on-DC

This project provides an Arduino sketch to control my model railway yard which consists of a number of parallel tracks where each track can consist of mutliple sections. 

Trains should approach a preselected track and will be stopped using "Brake on DC" when they reach a contact at the head of the track. If a track with already occupied sections is approached, the incoming train is stopped at the head right behind. 

Each track has a decoder address (MM2) assigned for control, pressing the *green* button on your control panel will release the train from the associated track and send it back on the track. If trains move out from tracks with multiple sections, trains from sections behind will be moved to the head of the track. The *red* button is currently used to reset the internal state based on the result of the contacts, it should not be required during normal operations.

## Control Workflow

1. On startup / after reset the first block is in `detect` state, all other blocks are in `forward` state. 
2. On activation of the break contact of the first block, it changes to `brake` state and sends the brake signal to the track by switching the relay to *on*.
3. The successor block is informed on the state change 
   * If the blocks break contact is set, it changes state to `follow` and also turns on the brake relay
   * if the break contact is not set, the block switches to the `detect` state
   * In both cases the next block is informed which behaves in the same way
4. On release (green button pressed), the front block switches to `moveout` and the track power is turned on. 
   * For all following blocks with mode `follow` the track power is also released, the state flag is **not** changed!
5. When the break contact of the front block changes to `open`, its state is changed to `detect` 
   * Following blocks with open contacts are set to `forward`
   * The first following block with a closed contact will be changed to `moveout`

If reset (red button) is called, the state of each block is re-evaluated:
   * Blocks with an open contact are put to `detect` with track power restored.
   * Blocks with a closed contact **not** in `follow` state are assumed to be in `break` state.
  * Blocks with a closed contact in `follow` state are assumed orphaned and moved to `brake` if the parent block is in `detect` 


## Hardware

To read from the digital control bus the sketch uses (a modified version of) the MaerklinMotorola library (https://www.arduino.cc/reference//en/libraries/maerklinmotorola/) and expects the Märklin signal on `Pin D2`. 

The track sensor for the first block is expected on `A0`, the corresponding relay must be attached to port `D3` - the following blocks use the subsequent ports. You have to defined the number of blocks and the layout with the heads address in the program code currently. The software does not make any limit on the number of blocks, it just counts up "by one" based on the literal names of the PINs so it might be easy to use more blocks with a larger Arduino or port extenders. 

### Inputs 

The inputs are read in analog mode and debounced in software (see DECAY settings). The internal PullUps are active, if you use A7/A8 on an Arduino Nano mind to add external PullUps! A `LOW` signal is evaluated as "train on track".

### Outputs

To not waste I/O PINs I decided to use mono-stable relays for the breaks. The default setting is "brake" (output is `LOW`), the relay control PIN goes to `HIGH` to let the train pass.



